'use strict';
window.browserGame=(()=>{
 const difficulties={easy:{step_limit:80,max_static_objects:4,max_pedestrians:2,max_observations:20},medium:{step_limit:120,max_static_objects:2,max_pedestrians:1,max_observations:30},hard:{step_limit:160,max_static_objects:1,max_pedestrians:0,max_observations:40}};
 const catalog=fetch('./data/maze_catalog.json').then(async r=>{if(!r.ok)throw Error('Maze catalog unavailable');return (await r.json()).mazes});
 let worker,ready,seq=0,episode,pending,running=false,skip=false,generation=0,backend='mujoco-wasm',wants3d=true;
 let phase='idle',lastFrame=null,wake=null;
 const signal=()=>{if(wake){const fn=wake;wake=null;fn()}};
 const wait=ms=>new Promise(resolve=>{let timer;const finish=()=>{clearTimeout(timer);if(wake===finish)wake=null;resolve()};wake=finish;if(ms!==undefined)timer=setTimeout(finish,ms)});
 const jobs=new Map();
 const status=s=>document.getElementById('browserStatus').textContent=s;
 const label=()=>backend==='mujoco-wasm'?'MUJOCO WASM · EXPERIMENTAL':'2D BROWSER PROTOTYPE';
 const note=()=>{document.getElementById('backendNote').textContent=backend==='mujoco-wasm'?(wants3d?'G1 physics · 3D replay':'G1 physics · 2D display'):'Grid proxy · not MuJoCo physics'};
 function warm(){
  if(ready)return ready;
  ready=new Promise((resolve,reject)=>{
   worker=new Worker('./engine-worker.js?v=10');
   worker.onmessage=({data})=>{
    if(data.type==='ready'){resolve();if(!running)status(label()+' · READY');return}
    if(data.type==='progress'){status(data.message);return}
    if(data.type==='error'){reject(Error(data.error));status('ENGINE LOAD FAILED · RELOAD');return}
    if(jobs.has(data.id)){const j=jobs.get(data.id);jobs.delete(data.id);data.error?j.reject(Error(data.error)):j.resolve(data.value)}
   };
   worker.onerror=e=>{const err=Error(e.message||'Engine error');reject(err);for(const j of jobs.values())j.reject(err);jobs.clear();status('ENGINE LOAD FAILED · RELOAD')};
  });ready.catch(()=>{});return ready;
 }
 const key=c=>c.join(',');
 function validate(maze,objects,peds,difficulty){
  const r=difficulties[difficulty],errors=[];
  if(!maze||!r)return {valid:false,errors:['Unknown maze or difficulty.']};
  if(objects.length>r.max_static_objects||peds.length>r.max_pedestrians)errors.push('Difficulty placement limit exceeded.');
  const used=new Set(),start=key(maze.start),goal=key(maze.goal);
  const free=c=>Array.isArray(c)&&c.length===2&&c.every(Number.isInteger)&&maze.wall_rows[c[1]]?.[c[0]]==='0';
  for(const c of [...objects,...peds]){if(!free(c)||key(c)===start||key(c)===goal||used.has(key(c)))errors.push('Place hazards on distinct open cells away from Start and Goal.');else used.add(key(c))}
  if(errors.length)return {valid:false,errors};
  const visited=new Set([start]),q=[maze.start];
  for(let i=0;i<q.length;i++){const [x,y]=q[i];for(const n of [[x+1,y],[x-1,y],[x,y+1],[x,y-1]]){const k=key(n);if(free(n)&&!used.has(k)&&!visited.has(k)){visited.add(k);q.push(n)}}}
  if(!visited.has(goal))errors.push('This layout blocks every grid route.');
  return {valid:!errors.length,errors}
 }
 function calculate(payload){return warm().then(()=>new Promise((resolve,reject)=>{const id=++seq;jobs.set(id,{resolve,reject});try{worker.postMessage({id,payload})}catch(e){jobs.delete(id);reject(e)}}))}
 function portraits(reaction='normal'){return calculate({task:'portraits',reaction})}
 async function api(url,options={}){
  const mazes=await catalog;
  if(url==='/api/catalog'){warm();return {mazes,difficulties}}
  const p=options.body?JSON.parse(options.body):{},maze=mazes.find(m=>m.id===p.maze_id);
  if(url==='/api/validate')return validate(maze,p.objects||[],p.pedestrians||[],p.difficulty);
  if(url==='/api/challenge'){
   if(running)return {ok:false,error:'Challenge already running.'};
   const v=validate(maze,p.objects||[],p.pedestrians||[],p.difficulty);
   if(!v.valid)return {ok:false,error:v.errors.join(' ')};
   if(!['periodic','age','uncertainty'].includes(p.strategy))return {ok:false,error:'Unknown strategy'};
   backend=p.backend==='mujoco-wasm'?'mujoco-wasm':'grid2d-proxy';wants3d=backend==='mujoco-wasm'&&!!p.render_3d;note();running=true;skip=false;episode=null;phase='running';lastFrame=null;generation++;pending=calculate({...p,task:'start',backend,render_3d:wants3d,maze,rules:difficulties[p.difficulty],seed:42000+generation});pending.catch(()=>{});return {ok:true};
  }
  if(url==='/api/pause'){
   if(!running||phase!=='running'||skip||!lastFrame)throw Error('Challenge is not available to pause.');
   phase='pausing';signal();return {ok:true};
  }
  if(url==='/api/validate-paused'||url==='/api/resume'){
   if(!running||phase!=='paused'||p.step!==lastFrame?.step)throw Error('The paused step changed.');
   const commit=url==='/api/resume';
   const verdict=await calculate({task:commit?'edit':'validate',layout:p});
   if(commit){if(!verdict.valid)throw Error(verdict.errors.join(' '));phase='running';signal();return {ok:true}}
   return verdict;
  }
  if(url==='/api/skip'){if(phase!=='running')throw Error('Resume the challenge first.');skip=true;signal();return {ok:true}}
  if(url==='/api/reset'){if(running)throw Error('Challenge still running');generation++;episode=null;return {ok:true}}
  if(url==='/api/review'){if(!episode)throw Error('No completed challenge');return {steps:episode.frames.map(f=>f.step),result:episode.result}}
  if(url.startsWith('/api/review/')){const f=episode?.frames.find(f=>f.step===Number(url.split('/').pop()));if(!f)throw Error('Frame unavailable');return f}
  throw Error('Unsupported local operation: '+url)
 }
 async function play(onFrame,onError){const token=generation;const frames=[];
  let initialized3d=false,warned=false;
  try{
   status(label()+' · LOADING');await pending;window.g1View.clear();status(label()+' · RUNNING');
   while(token===generation){
    if(phase==='pausing'){
     phase='paused';status(label()+' · PAUSED');
     onFrame({...lastFrame,status:'paused'});
    }
    if(phase==='paused'){await wait();continue}
    const data=await calculate({task:'advance',count:skip?16:1});
    if(token!==generation)return;
    if(wants3d&&data.renderAssets){
     if(!initialized3d){initialized3d=await window.g1View.prepare(data.renderAssets,data.sceneRevision)}
     else window.g1View.register(data.sceneRevision,data.renderAssets);
    }
    if(wants3d&&data.renderError&&!warned){warned=true;console.warn(data.renderError);document.getElementById('backendNote').title=data.renderError;toast('3D unavailable: '+data.renderError)}
    if(data.walkWarning)document.getElementById('backendNote').title='Standing-pose fallback: '+data.walkWarning;
    for(const frame of data.frames){frames.push(frame);lastFrame=frame;if(!skip)onFrame(frame)}
    if(data.result){
     if(data.result.backend!==backend)throw Error('Unexpected physics backend');
     episode={frames,result:data.result};running=false;phase='finished';status(label()+' · READY');
     onFrame({...lastFrame,status:'finished',result:data.result});return;
    }
    if(phase==='running'&&!skip)await wait(130);
   }
  }catch(e){running=false;phase='idle';try{await calculate({task:'abort'})}catch(_){}status(label()+' · ERROR: '+e.message);onError(e)}
 }
 return {api,play,validate,portraits}
})();
