"""Experimental browser 2D proxy, not MuJoCo."""
import json, hashlib, random, math
from spatial_memory.simulation import Scenario
from spatial_memory.evaluation import iter_episode
from spatial_memory.navigation import HEADINGS, heading_index

def _free_set(maze: dict) -> set[tuple[int, int]]:
    return {(x, y) for y, row in enumerate(maze['wall_rows'])
            for x, bit in enumerate(row) if bit == '0'}


def make_scenario(maze: dict, objects: list[list[int]], ped_paths: list[list[list[int]]]) -> Scenario:
    walls = [(x, y) for y, row in enumerate(maze['wall_rows'])
             for x, bit in enumerate(row) if bit == '1']
    return Scenario.from_dict({
        'name': f"BYU-AI-Day-2026:{maze['id']}", 'split': 'development',
        'width': maze['width'], 'height': maze['height'],
        'start': maze['start'], 'goal': maze['goal'], 'walls': walls,
        'obstacles': [tuple(x) for x in objects],
        'pedestrians': [[tuple(x) for x in path] for path in ped_paths],
        'changes': [],
    })

def make_pedestrian_paths(maze: dict, starts: list[list[int]], objects: list[list[int]],
                          horizon: int = 80) -> list[list[list[int]]]:
    """Seeded, collision-free cardinal wandering for the full exhibition horizon.

    Avoid immediate reversals when alternatives exist, prefer less visited
    cells, exclude the start/goal and objects, and avoid simultaneous
    pedestrian overlaps or exchanges. Append a reverse trace solely to
    satisfy Scenario's cyclic adjacency contract (not played in the episode).
    """
    if not starts:
        return []
    free = _free_set(maze) - {tuple(x) for x in objects} - {tuple(maze['start']), tuple(maze['goal'])}
    seed_data = json.dumps([maze['id'], sorted(map(tuple, objects)), starts],
                           separators=(',', ':')).encode()
    rng = random.Random(int.from_bytes(hashlib.sha256(seed_data).digest()[:8], 'big'))
    positions = [tuple(cell) for cell in starts]
    paths = [[cell] for cell in positions]
    visits = [{cell: 1} for cell in positions]
    previous = [None for _ in positions]
    for step in range(horizon):
        old = positions.copy()
        order = list(range(len(old)))
        rng.shuffle(order)
        reserved = set(old)
        for i in order:
            x, y = old[i]
            candidates = [q for q in ((x+1,y),(x-1,y),(x,y+1),(x,y-1))
                          if q in free and q not in reserved]
            if any(q != previous[i] for q in candidates):
                candidates = [q for q in candidates if q != previous[i]]
            target = (rng.choices(candidates,
                       weights=[1/(1+visits[i].get(q,0)) for q in candidates], k=1)[0]
                      if candidates else old[i])
            # Keep all previous cells reserved until the next tick: no swaps.
            reserved.add(target)
            positions[i] = target
        for i, pos in enumerate(positions):
            paths[i].append(pos)
            visits[i][pos] = visits[i].get(pos, 0) + 1
            if pos != old[i]:
                previous[i] = old[i]
    return [[list(q) for q in path + path[-2:0:-1]] for path in paths]

class FootprintReachability:
    """Conservative state-space reachability from actual G1 collision bounds."""
    CELL_M = 0.75
    WALL_HALF_CELL = 0.505

    def __init__(self, half_extents: tuple[float, float], parts=None):
        self.hx, self.hy = map(float, half_extents)
        self.parts = tuple(parts or ())
        self._offset_cache = {}

    @classmethod
    def from_pinned_g1(cls) -> 'FootprintReachability':
        # The small public calibration scene uses the committed G1 MJCF only.
        size = 7
        walls = {(x, y) for x in range(size) for y in range(size)
                 if x in (0, size-1) or y in (0, size-1)}
        scenario = Scenario.from_dict(dict(name='ai-day-footprint-calibration', split='development',
            width=size, height=size, start=(2, 3), goal=(4, 3), walls=sorted(walls),
            obstacles=[], pedestrians=[], changes=[]))
        scene = MujocoNavigationScene(scenario, cell_size=cls.CELL_M)
        root = scene.data.qpos[scene.robot_qpos_adr:scene.robot_qpos_adr+2].copy()
        parts = []
        for geom in sorted(scene._robot_geom_ids):
            world_xy = scene.data.geom_xpos[geom, :2] - root
            local_xy = tuple(map(float, world_xy))
            radius = float(scene.model.geom_rbound[geom])
            if math.isfinite(radius) and radius > 0:
                parts.append((local_xy[0], local_xy[1], radius))
        return cls(scene.footprint_half_extents_local, parts)

    def _clear(self, cell: tuple[float, float], yaw: float,
               occupied: set[tuple[int, int]], width: int, height: int) -> bool:
        x, y = cell
        if self.parts:
            base_x, base_y = math.floor(x), math.floor(y)
            fx, fy = round(x-base_x, 1), round(y-base_y, 1)
            key = (float(yaw), fx, fy)
            offsets = self._offset_cache.get(key)
            if offsets is None:
                c, s = math.cos(yaw), math.sin(yaw)
                half = self.WALL_HALF_CELL*self.CELL_M
                found = []
                for ox in range(-2,3):
                    for oy in range(-2,3):
                        for lx, ly, radius in self.parts:
                            px = fx*self.CELL_M + c*lx - s*ly
                            py = fy*self.CELL_M + s*lx + c*ly
                            dx = max(abs(px-ox*self.CELL_M)-half, 0.0)
                            dy = max(abs(py-oy*self.CELL_M)-half, 0.0)
                            if dx*dx + dy*dy <= radius*radius:
                                found.append((ox,oy)); break
                offsets = self._offset_cache[key] = tuple(found)
            if any((base_x+ox,base_y+oy) in occupied for ox,oy in offsets):
                return False
            # Keep the center and the public collision envelope inside the map.
            if (x <= 0 or y <= 0 or x >= width-1 or y >= height-1):
                return False
            return True
        c, s = abs(math.cos(yaw)), abs(math.sin(yaw))
        hx, hy = c*self.hx + s*self.hy, s*self.hx + c*self.hy
        rx = hx/self.CELL_M + self.WALL_HALF_CELL
        ry = hy/self.CELL_M + self.WALL_HALF_CELL
        # Treat outside-map cells as solid, including the actual wall-box half width.
        if x-rx <= -0.5 or y-ry <= -0.5 or x+rx >= width-0.5 or y+ry >= height-0.5:
            return False
        for ox, oy in occupied:
            dx, dy = abs(x-ox)*self.CELL_M, abs(y-oy)*self.CELL_M
            if dx <= hx + self.WALL_HALF_CELL*self.CELL_M and dy <= hy + self.WALL_HALF_CELL*self.CELL_M:
                return False
        return True

    def _turn_clear(self, cell, from_heading, to_heading, occupied, width, height):
        start, end = HEADINGS[from_heading], HEADINGS[to_heading]
        delta = (end-start+math.pi) % (2*math.pi)-math.pi
        samples = max(1, math.ceil(abs(delta)/0.05))
        return all(self._clear(cell, start+delta*i/samples, occupied, width, height)
                   for i in range(1, samples+1))

    def has_route(self, maze: dict, objects: list[list[int]], *, start_heading: int = 0) -> bool:
        width, height = maze['width'], maze['height']
        occupied = {(x, y) for y, row in enumerate(maze['wall_rows'])
                    for x, bit in enumerate(row) if bit == '1'} | {tuple(x) for x in objects}
        start, goal = tuple(maze['start']), tuple(maze['goal'])
        pose_cache = {}
        def clear(cell, yaw):
            key = (round(float(cell[0]), 3), round(float(cell[1]), 3), round(float(yaw), 3))
            if key not in pose_cache:
                pose_cache[key] = self._clear(cell, yaw, occupied, width, height)
            return pose_cache[key]
        turn_cache = {}
        def turn_clear(cell, h0, h1):
            key = (cell, h0, h1)
            if key not in turn_cache:
                start_yaw, end_yaw = HEADINGS[h0], HEADINGS[h1]
                delta = (end_yaw-start_yaw+math.pi) % (2*math.pi)-math.pi
                samples = max(1, math.ceil(abs(delta)/0.05))
                turn_cache[key] = all(clear(cell, start_yaw+delta*i/samples)
                                      for i in range(1, samples+1))
            return turn_cache[key]
        queue = [(start[0], start[1], start_heading)]
        seen = set(queue)
        cursor = 0
        while cursor < len(queue):
            x, y, heading = queue[cursor]; cursor += 1
            cell = (x, y)
            if cell == goal:
                return True
            for new_heading in range(4):
                if new_heading == heading or not turn_clear(cell, heading, new_heading):
                    continue
                state = (x, y, new_heading)
                if state not in seen:
                    seen.add(state); queue.append(state)
            dx, dy = ((1,0),(0,1),(-1,0),(0,-1))[heading]
            target = (x+dx, y+dy)
            if target in occupied or not clear(target, HEADINGS[heading]):
                continue
            # Check the actual swept centerline at the shield's 1/10-cell granularity.
            if not all(clear((x+dx*i/10, y+dy*i/10), HEADINGS[heading])
                       for i in range(1, 11)):
                continue
            state = (target[0], target[1], heading)
            if state not in seen:
                seen.add(state); queue.append(state)
        return False



CONFIG = json.loads("{\"common_horizon_metrics\":true,\"g1_navigation\":{\"footprint_source\":\"pinned G1 collision geometry\",\"forward_translation_default\":true,\"local_half_extents_m\":[0.218954831277208,0.3279970912029507],\"motion\":\"cardinal kinematic translation with separate in-place turns\",\"observed_occupancy_only\":true,\"pedestrian_prediction\":\"motion inferred only from observed first-hit cells; no simulator schedule or future positions\",\"rejected_action_history\":true,\"retreat_eligibility\":\"recent single-cluster pedestrian detections in two observations indicate imminent approach; entire backward sweep observed free, otherwise wait\",\"sidestep_eligibility\":\"around a measured persistent obstacle, or one lateral escape from a close observed pedestrian threat; complete swept clearance observed free and only the selected action is shield-checked\",\"turn_clearance\":\"deferred_to_selected_action_mujoco_shield; no AABB prefilter; one selected turn checked, rejection returned as planner feedback\"},\"max_observations\":20,\"max_steps\":80,\"mujoco_cell_size_m\":0.75,\"mujoco_ray_sensor\":{\"horizontal_fov_deg\":120,\"range_m\":6,\"ray_count\":61,\"sample_spacing_m\":0.075},\"mujoco_timestep_seconds\":0.02,\"observation_cost\":1,\"sensor\":{\"noise\":0,\"occlusion\":true,\"radius\":3},\"sensor_backend\":\"mujoco3d\",\"unknown\":\"optimistic\",\"update_interval\":4}")

def _queue_environment(world, q, ctx, objects, pedestrians, replace_scene=None):
    """Apply only at the next real evaluator tick; never write agent memory."""
    paths=make_pedestrian_paths(q['maze'],pedestrians,objects,q['rules']['step_limit']-world.step)
    paths=[[path[0]]*world.step+path for path in paths]
    replacement=make_scenario(q['maze'],objects,paths)
    advance=world.advance
    def apply_and_advance():
        world.advance=advance
        if replace_scene is not None:
            replace_scene(replacement)
        world.scenario=replacement
        world.objects=set(replacement.obstacles)
        ctx['revision']+=1
        advance()
    world.advance=apply_and_advance

def _register_frame(ctx, world, history, apply):
    ctx['frame']=history[-1]
    ctx['frame']['scene_revision']=ctx['revision']
    ctx['apply']=apply

def grid_episode(request_json, ctx):
    q=json.loads(request_json)
    m=q['maze']
    objects=q['objects']
    rules=q['rules']
    scenario=make_scenario(m, objects, make_pedestrian_paths(m,q['pedestrians'],objects,rules['step_limit']))
    config=dict(CONFIG,max_steps=rules['step_limit'],max_observations=rules['max_observations'],common_horizon_metrics=False,sensor_backend='grid2d')
    history=[]
    trajectory=[]
    events=[]
    observed_objects=set()
    observed_people=set()
    def capture(world, frame):
        pos=tuple(frame['position'])
        prior=tuple(trajectory[-1]) if trajectory else pos
        people=world.people()
        prev=set(map(tuple,history[-1]['people'])) if history else people
        hit=int(bool(history) and prior in people and prior not in prev)
        contacts={'g1_pedestrian_contact_pairs':hit}
        if hit:
            events.append(dict(step=int(world.step),contacts=contacts))
        trajectory.append(list(pos))
        memory=frame['memory']
        if frame['observation_granted']:
            observed_objects.update(c for c in world.objects if memory[c[1],c[0]]==1)
            observed_people.clear()
            observed_people.update(c for c in people if memory[c[1],c[0]]==1)
        yaw=math.atan2(pos[1]-prior[1],pos[0]-prior[0]) if pos!=prior else (history[-1]['heading_yaw'] if history else 0.0)
        history.append(dict(status='running',frame_seq=len(history),step=int(world.step),maze_id=m['id'],strategy=q['strategy'],difficulty=q['difficulty'],position=list(pos),heading_yaw=yaw,memory=memory.tolist(),geometry=frame['geometry'].astype(int).tolist(),objects=[list(c) for c in sorted(world.objects)],people=[list(c) for c in sorted(people)],trajectory=list(trajectory),observed_objects=[list(c) for c in sorted(observed_objects)],observed_people=[list(c) for c in sorted(observed_people)],observations=int(frame['observation_count']),contacts=contacts,contact_events=list(events),step_limit=rules['step_limit'],max_observations=rules['max_observations']))
        _register_frame(ctx,world,history,lambda o,p: _queue_environment(world,q,ctx,o,p))
    metrics,_=yield from iter_episode(scenario,config,q['strategy'],seed=q['seed'],_frame_callback=capture)
    result=dict(winner='AI WINS' if metrics['success'] and not events else 'HUMAN WINS',success=bool(metrics['success']),elapsed_steps=int(metrics['elapsed_steps']),observation_count=int(metrics['observation_count']),max_observations=rules['max_observations'],step_limit=rules['step_limit'],maze_id=m['id'],strategy=q['strategy'],difficulty=q['difficulty'],actual_unwanted_contacts=len(events),contact_events=events,blocked_attempts_not_contacts=int(metrics['blocked_movement_attempts']),backend='grid2d-proxy')
    return dict(frames=history,result=result)


def wasm_episode(request_json, ctx):
    """Use the same evaluator, forward-first planner, ray sensors and action shield.
    The browser supplies the native API subset through mujoco.py.
    """
    from spatial_memory.mujoco_navigation import MujocoNavigationScene
    from spatial_memory.mujoco_perception import MujocoRayPerception
    from spatial_memory.emergency_safety import IndependentEmergencySafetySensor, EmergencySafetyController
    from spatial_memory.pedestrian_tracking import ObservedPedestrianTracker
    from spatial_memory.navigation import HEADINGS, heading_index, plan_g1_default_policy

    q=json.loads(request_json)
    m=q['maze']
    rules=q['rules']
    scenario=make_scenario(m,q['objects'],make_pedestrian_paths(m,q['pedestrians'],q['objects'],rules['step_limit']))
    config=json.loads(json.dumps(CONFIG))
    config.update(max_steps=rules['step_limit'],max_observations=rules['max_observations'],common_horizon_metrics=False)
    scene=MujocoNavigationScene(scenario,cell_size=config['mujoco_cell_size_m'],timestep=config['mujoco_timestep_seconds'])
    memory_sensor=MujocoRayPerception(scene,**config['mujoco_ray_sensor'])
    safety_sensor=IndependentEmergencySafetySensor(scene,range_m=2.0,ray_count=61,sample_spacing_m=0.075,ray_backend='multi')
    safety=EmergencySafetyController(scene)
    tracker=ObservedPedestrianTracker(max_age_steps=8)
    state=dict(static=set(),observed_step=-1,planner_step=0)
    history=[]
    trajectory=[]
    events=[]
    seen_objects=set()
    seen_people=set()

    def planner(grid,position,goal,unknown,yaw,blocked):
        state['planner_step']+=1
        recent=state['static'] if 0<=state['planner_step']-state['observed_step']<=8 else set()
        return plan_g1_default_policy(grid,position,goal,heading=heading_index(yaw),
            unknown=unknown,local_extents=scene.footprint_half_extents_local,cell_size=scene.cell_size,
            blocked_actions=blocked,static_obstacle_cells=recent,pedestrian_context=tracker.context(),step=state['planner_step'])

    def record(_obs,step,_position):
        stats=memory_sensor.last_stats
        tracker.observe(step,stats.get('visible_pedestrian_cells',()))
        state['static']={tuple(c) for c in stats.get('visible_persistent_obstacle_cells',())}
        state['observed_step']=int(step)
        seen_objects.update(state['static'])
        seen_people.clear()
        seen_people.update(tuple(c) for c in stats.get('visible_pedestrian_cells',()))

    def checker(world,position,action,yaw):
        kind,value=action
        if kind in {'move','sidestep','retreat'}:
            return scene.check_move(world,position,tuple(value),yaw,target_yaw=yaw)
        return scene.check_turn(world,position,yaw,HEADINGS[int(value)])

    def replace_scene(next_scenario):
        nonlocal scene
        scene=MujocoNavigationScene(next_scenario,cell_size=scene.cell_size,timestep=scene.timestep)
        memory_sensor.scene=scene
        safety_sensor.scene=scene

    from js import wasmBridge
    render_3d=bool(q.get('render_3d',False))
    render_failure=None
    walk_phase=0
    walk_failure=None
    def capture(world,frame):
        nonlocal render_failure, walk_phase, walk_failure
        extra=scene.update(world,frame)
        contacts={k:int(extra.get(k,0)) for k in ('g1_pedestrian_contact_pairs','g1_wall_contact_pairs',
            'g1_persistent_obstacle_contact_pairs','g1_unwanted_floor_contact_pairs','g1_self_contact_pairs')}
        if any(contacts.values()):
            events.append(dict(step=int(world.step),contacts=contacts,backend='mujoco-wasm'))
        pos=list(map(int,frame['position']))
        trajectory.append(pos)
        history.append(dict(status='running',frame_seq=len(history),step=int(world.step),
            maze_id=m['id'],strategy=q['strategy'],difficulty=q['difficulty'],position=pos,
            heading_yaw=float(frame['heading_yaw']),memory=frame['memory'].astype(int).tolist(),
            geometry=frame['geometry'].astype(int).tolist(),objects=[list(c) for c in sorted(world.objects)],
            people=[list(c) for c in sorted(world.people())],trajectory=list(trajectory),
            observed_objects=[list(c) for c in sorted(seen_objects)],observed_people=[list(c) for c in sorted(seen_people)],
            observations=int(frame['observation_count']),contacts=contacts,contact_events=list(events),
            step_limit=rules['step_limit'],max_observations=rules['max_observations'],backend='mujoco-wasm'))
        if render_3d and render_failure is None:
            try:
                moved=bool(frame.get('moved',False))
                if moved:
                    walk_phase+=1
                if walk_failure is None:
                    try:
                        history[-1]['render_pose']=json.loads(wasmBridge.walkPose(walk_phase,moved))
                    except Exception as error:
                        walk_failure=str(error)
                if 'render_pose' not in history[-1]:
                    history[-1]['render_pose']=json.loads(wasmBridge.renderPose())
            except Exception as error:
                render_failure=str(error)
        if ctx.get('reach') is None:
            root=scene.data.qpos[scene.robot_qpos_adr:scene.robot_qpos_adr+2].copy()
            parts=[]
            for geom in sorted(scene._robot_geom_ids):
                xy=scene.data.geom_xpos[geom,:2]-root
                radius=float(scene.model.geom_rbound[geom])
                if math.isfinite(radius) and radius>0:
                    parts.append((float(xy[0]),float(xy[1]),radius))
            ctx['reach']=FootprintReachability(scene.footprint_half_extents_local,parts)
        _register_frame(ctx,world,history,lambda o,p: _queue_environment(world,q,ctx,o,p,replace_scene))
        ctx['renderError']=render_failure
        ctx['walkWarning']=walk_failure
        return extra

    metrics,_=yield from iter_episode(scenario,config,q['strategy'],seed=q['seed'],_frame_callback=capture,
        _observation_provider=memory_sensor.observe,_g1_planner=planner,_action_checker=checker,
        _initial_observation_callback=record,_observation_callback=record,
        _emergency_safety_sensor=safety_sensor.scan,_emergency_safety_controller=safety.choose)
    actual=int(metrics.get('g1_unwanted_contact_pairs',0))
    if actual!=sum(sum(event['contacts'].values()) for event in events):
        raise RuntimeError('Contact count mismatch in WASM adapter')
    result=dict(winner='AI WINS' if metrics['success'] and actual==0 else 'HUMAN WINS',
        success=bool(metrics['success']),elapsed_steps=int(metrics['elapsed_steps']),observation_count=int(metrics['observation_count']),
        max_observations=rules['max_observations'],step_limit=rules['step_limit'],maze_id=m['id'],strategy=q['strategy'],
        difficulty=q['difficulty'],actual_unwanted_contacts=actual,contact_events=events,
        blocked_attempts_not_contacts=int(metrics.get('g1_footprint_blocked_attempts',0)),
        backend='mujoco-wasm',footprint_half_extents_m=list(scene.footprint_half_extents_local),
        emergency_safety_scans=int(metrics.get('emergency_safety_scans',0)))
    return dict(frames=history,result=result,renderError=render_failure,walkWarning=walk_failure)

# Session state belongs to one worker, and the Python generator is suspended
# between steps. The browser never restarts or replays an agent to apply edits.
_session=None

def _dump(value):
    return json.dumps(value,allow_nan=False)

def session_start(request_json):
    global _session
    if _session is not None:
        raise RuntimeError('An evaluation is already active')
    q=json.loads(request_json)
    ctx=dict(q=q,revision=0,frame=None,reach=None,edits=[])
    ctx['iterator']=(wasm_episode if q.get('backend')=='mujoco-wasm' else grid_episode)(request_json,ctx)
    _session=ctx
    return _dump({'ok':True})

def session_close():
    global _session
    if _session is not None:
        _session['iterator'].close()
        _session=None

def session_next(count=1):
    ctx=_session
    if ctx is None:
        raise RuntimeError('No active evaluation')
    frames=[]
    for i in range(max(1,int(count))):
        next(ctx['iterator'])
        f=ctx['frame']
        frames.append(f)
        terminal=tuple(f['position'])==tuple(ctx['q']['maze']['goal']) or f['step']>=ctx['q']['rules']['step_limit']
        if terminal:
            try:
                next(ctx['iterator'])
                raise RuntimeError('Unexpected frame after terminal step')
            except StopIteration as done:
                data=done.value
                data['result']['edit_events']=ctx['edits']
                return _dump(dict(frames=frames,result=data['result'],sceneRevision=ctx['revision'],
                    renderError=ctx.get('renderError'),walkWarning=ctx.get('walkWarning')))
    return _dump(dict(frames=frames,sceneRevision=ctx['revision'],renderError=ctx.get('renderError'),walkWarning=ctx.get('walkWarning')))

def _validate_edit(ctx, request):
    f=ctx['frame']
    if f is None or request.get('step')!=f['step']:
        return {'valid':False,'errors':['The paused step changed.']}
    q=ctx['q']; m=q['maze']; rules=q['rules']
    objects=request['objects']; pedestrians=request['pedestrians']
    def fail(message): return {'valid':False,'errors':[message]}
    if len(objects)>rules['max_static_objects'] or len(pedestrians)>rules['max_pedestrians']:
        return fail('Placement limit exceeded. Remove an existing hazard first.')
    if any(not isinstance(c,list) or len(c)!=2 or any(type(v) is not int for v in c) for c in objects+pedestrians):
        return fail('Placement must be a grid cell.')
    o,p=set(map(tuple,objects)),set(map(tuple,pedestrians))
    if len(o)!=len(objects) or len(p)!=len(pedestrians) or o&p:
        return fail('Hazards cannot overlap.')
    if not (o|p)<=_free_set(m) or (o|p)&{tuple(m['start']),tuple(m['goal'])}:
        return fail('Use open cells outside Start and Goal.')
    added=(o-set(map(tuple,f['objects'])))|(p-set(map(tuple,f['people'])))
    if added:
        pos=tuple(f['position']); yaw=f['heading_yaw']
        if pos in added:
            return fail('Keep new hazards clear of the humanoid.')
        reach=ctx['reach']
        if reach is not None:
            if not reach._clear(pos,yaw,added,m['width'],m['height']):
                return fail('Keep new hazards clear of the humanoid.')
            if not reach.has_route(dict(m,start=list(pos)),sorted(o|p),start_heading=heading_index(yaw)):
                return fail('This layout blocks every footprint-safe route to Goal.')
        else:
            free=_free_set(m)-(o|p)
            visited={pos}; queue=[pos]
            for x,y in queue:
                for cell in ((x+1,y),(x-1,y),(x,y+1),(x,y-1)):
                    if cell in free and cell not in visited:
                        visited.add(cell);queue.append(cell)
            if tuple(m['goal']) not in visited:
                return fail('This layout blocks every grid route to Goal.')
    return {'valid':True,'errors':[]}

def session_edit(request_json, commit=False):
    ctx=_session
    if ctx is None: raise RuntimeError('No active evaluation')
    request=json.loads(request_json)
    verdict=_validate_edit(ctx,request)
    if not verdict['valid'] or not commit:
        return _dump(verdict)
    f=ctx['frame']
    o=sorted(set(map(tuple,request['objects'])))
    p=sorted(set(map(tuple,request['pedestrians'])))
    if set(o)!=set(map(tuple,f['objects'])) or set(p)!=set(map(tuple,f['people'])):
        objects=[list(c) for c in o];people=[list(c) for c in p]
        ctx['apply'](objects,people)
        ctx['edits'].append(dict(paused_step=f['step'],effective_step=f['step']+1,
            objects_before=f['objects'],objects_after=objects,pedestrians_before=f['people'],pedestrians_after=people))
    return _dump(verdict)

def browser_episode(request_json):
    # Non-interactive compatibility entry point.
    session_start(request_json)
    frames=[]
    try:
        while True:
            data=json.loads(session_next(16));frames.extend(data['frames'])
            if 'result' in data:
                data['frames']=frames
                return _dump(data)
    finally:
        session_close()
