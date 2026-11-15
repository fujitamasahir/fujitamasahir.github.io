const CDN='https://cdn.jsdelivr.net/pyodide/v0.27.7/full/';
const files=['__init__','simulation','observation','scheduling','memory','navigation','evaluation','mujoco_navigation','mujoco_perception','emergency_safety','pedestrian_tracking'].map(n=>'spatial_memory/'+n+'.py').concat('browser_engine.py','mujoco.py','portrait_poses.py');
const ready=(async()=>{
 importScripts(CDN+'pyodide.js','./mujoco-bridge.js?v=9');
 const p=await loadPyodide({indexURL:CDN});
 await p.loadPackage('numpy');p.FS.mkdirTree('/game/spatial_memory');
 await Promise.all(files.map(async file=>{const r=await fetch('./python/'+file+'?v=9');if(!r.ok)throw Error('Missing '+file);p.FS.writeFile('/game/'+file,await r.text())}));
 p.runPython("import sys; sys.path.insert(0, '/game'); from browser_engine import session_start, session_next, session_edit, session_close");return p;
})();
let nativeReady;
async function prepareNative(p){
 if(!nativeReady)nativeReady=(async()=>{
  const [r,m]=await Promise.all([fetch('./models/g1.xml'),fetch('./models/manifest.json')]);
  if(!r.ok||!m.ok)throw Error('G1 XML or manifest missing');
  const bytes=new Uint8Array(await r.arrayBuffer()),manifest=await m.json();
  const digest=Array.from(new Uint8Array(await crypto.subtle.digest('SHA-256',bytes)),x=>x.toString(16).padStart(2,'0')).join('');
  if(digest!==manifest.xml_sha256)throw Error('G1 XML SHA-256 mismatch');
  p.FS.mkdirTree('/third_party/unitree_g1');
  p.FS.writeFile('/third_party/unitree_g1/g1.xml',bytes);
  await wasmBridge.prepare();
 })();
 return nativeReady;
}
ready.then(()=>postMessage({type:'ready'}),e=>postMessage({type:'error',error:String(e)}));
let active=null,exportedRevision=-1;
function invoke(p,name,...args){const fn=p.globals.get(name);try{return fn(...args)}finally{fn.destroy()}}
function closeSession(p){try{invoke(p,'session_close')}finally{wasmBridge.close();active=null;exportedRevision=-1}}
async function handle(data){
 const payload=data.payload,task=payload.task;
 const p=await ready;
 try{
  if(task==='portraits'){
   if(active)throw Error('Portrait generation cannot interrupt a live evaluation');
   await prepareNative(p);
   try{
    p.runPython('from portrait_poses import portrait_bundle');
    const value=JSON.parse(invoke(p,'portrait_bundle',payload.reaction)),transfer=[];
    if(payload.reaction==='normal'){
     value.renderAssets=wasmBridge.renderAssets();
     for(const m of value.renderAssets.meshes)transfer.push(m.positions.buffer,m.indices.buffer);
     if(value.still.length!==value.renderAssets.poseLength||Object.values(value.clips).some(clip=>clip.length!==60||clip.some(f=>f.length!==value.renderAssets.poseLength)))throw Error('Portrait dimensions mismatch');
    }
    postMessage({id:data.id,value},transfer);
   }finally{wasmBridge.close()}
   return;
  }
  if(task==='start'){
   if(active)throw Error('An evaluation is already active');
   if(payload.backend==='mujoco-wasm'){postMessage({type:'progress',message:'MUJOCO WASM · LOADING G1'});await prepareNative(p)}
   const value=JSON.parse(invoke(p,'session_start',JSON.stringify(payload)));
   active=payload;exportedRevision=-1;postMessage({id:data.id,value});return;
  }
  if(!active)throw Error('No active evaluation');
  if(task==='validate'||task==='edit'){
   const value=JSON.parse(invoke(p,'session_edit',JSON.stringify(payload.layout),task==='edit'));
   postMessage({id:data.id,value});return;
  }
  if(task==='advance'){
   const value=JSON.parse(invoke(p,'session_next',payload.count||1)),transfer=[];
   if(active.render_3d&&!value.renderError&&exportedRevision!==value.sceneRevision){
    try{
     value.renderAssets=wasmBridge.renderAssets();
     if(value.frames.some(f=>f.render_pose?.length!==value.renderAssets.poseLength))throw Error('Render pose shape mismatch');
     for(const m of value.renderAssets.meshes)transfer.push(m.positions.buffer,m.indices.buffer);
     exportedRevision=value.sceneRevision;
    }catch(e){delete value.renderAssets;transfer.length=0;value.renderError=String(e)}
   }
   if(value.result)closeSession(p);
   postMessage({id:data.id,value},transfer);return;
  }
  if(task==='abort'){closeSession(p);postMessage({id:data.id,value:{ok:true}});return}
  throw Error('Unknown game worker task');
 }catch(e){
  if(task==='start'||task==='advance'||task==='abort'){if(active)closeSession(p);else wasmBridge.close()}
  throw e;
 }
}
let queue=Promise.resolve();self.onmessage=({data})=>{queue=queue.then(()=>handle(data)).catch(error=>postMessage({id:data.id,error:String(error)}))};
