"""Experimental browser 2D proxy, not MuJoCo."""
import json, hashlib, random, math
from spatial_memory.simulation import Scenario
from spatial_memory.evaluation import run_episode

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

CONFIG = json.loads("{\"common_horizon_metrics\":true,\"g1_navigation\":{\"footprint_source\":\"pinned G1 collision geometry\",\"forward_translation_default\":true,\"local_half_extents_m\":[0.218954831277208,0.3279970912029507],\"motion\":\"cardinal kinematic translation with separate in-place turns\",\"observed_occupancy_only\":true,\"pedestrian_prediction\":\"motion inferred only from observed first-hit cells; no simulator schedule or future positions\",\"rejected_action_history\":true,\"retreat_eligibility\":\"recent single-cluster pedestrian detections in two observations indicate imminent approach; entire backward sweep observed free, otherwise wait\",\"sidestep_eligibility\":\"around a measured persistent obstacle, or one lateral escape from a close observed pedestrian threat; complete swept clearance observed free and only the selected action is shield-checked\",\"turn_clearance\":\"deferred_to_selected_action_mujoco_shield; no AABB prefilter; one selected turn checked, rejection returned as planner feedback\"},\"max_observations\":20,\"max_steps\":80,\"mujoco_cell_size_m\":0.75,\"mujoco_ray_sensor\":{\"horizontal_fov_deg\":120,\"range_m\":6,\"ray_count\":61,\"sample_spacing_m\":0.075},\"mujoco_timestep_seconds\":0.02,\"observation_cost\":1,\"sensor\":{\"noise\":0,\"occlusion\":true,\"radius\":3},\"sensor_backend\":\"mujoco3d\",\"unknown\":\"optimistic\",\"update_interval\":4}")

def grid_episode(request_json):
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
        history.append(dict(status='running',frame_seq=len(history),step=int(world.step),maze_id=m['id'],strategy=q['strategy'],difficulty=q['difficulty'],position=list(pos),heading_yaw=yaw,memory=memory.tolist(),geometry=frame['geometry'].astype(int).tolist(),objects=objects,people=[list(c) for c in sorted(people)],trajectory=list(trajectory),observed_objects=[list(c) for c in sorted(observed_objects)],observed_people=[list(c) for c in sorted(observed_people)],observations=int(frame['observation_count']),contacts=contacts,contact_events=list(events),step_limit=rules['step_limit'],max_observations=rules['max_observations']))
    metrics,_=run_episode(scenario,config,q['strategy'],seed=q['seed'],_frame_callback=capture)
    result=dict(winner='AI WINS' if metrics['success'] and not events else 'HUMAN WINS',success=bool(metrics['success']),elapsed_steps=int(metrics['elapsed_steps']),observation_count=int(metrics['observation_count']),max_observations=rules['max_observations'],step_limit=rules['step_limit'],maze_id=m['id'],strategy=q['strategy'],difficulty=q['difficulty'],actual_unwanted_contacts=len(events),contact_events=events,blocked_attempts_not_contacts=int(metrics['blocked_movement_attempts']),backend='grid2d-proxy')
    return json.dumps(dict(frames=history,result=result),allow_nan=False)


def wasm_episode(request_json):
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

    from js import wasmBridge
    render_3d=bool(q.get('render_3d',False))
    def capture(world,frame):
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
        if render_3d:
            history[-1]['render_pose']=json.loads(wasmBridge.renderPose())
        return extra

    metrics,_=run_episode(scenario,config,q['strategy'],seed=q['seed'],_frame_callback=capture,
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
    return json.dumps(dict(frames=history,result=result),allow_nan=False)

def browser_episode(request_json):
    if json.loads(request_json).get('backend')=='mujoco-wasm':
        return wasm_episode(request_json)
    return grid_episode(request_json)
