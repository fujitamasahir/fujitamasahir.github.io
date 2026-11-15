const CDN='https://cdn.jsdelivr.net/pyodide/v0.27.7/full/';
const files=['__init__','simulation','observation','scheduling','memory','navigation','evaluation','mujoco_navigation','mujoco_perception','emergency_safety','pedestrian_tracking'].map(n=>'spatial_memory/'+n+'.py').concat('browser_engine.py','mujoco.py','portrait_poses.py');
const ready=(async()=>{
 importScripts(CDN+'pyodide.js','./mujoco-bridge.js?v=5');
 const p=await loadPyodide({indexURL:CDN});
 await p.loadPackage('numpy');p.FS.mkdirTree('/game/spatial_memory');
 await Promise.all(files.map(async file=>{const r=await fetch('./python/'+file+'?v=5');if(!r.ok)throw Error('Missing '+file);p.FS.writeFile('/game/'+file,await r.text())}));
 p.runPython("import sys; sys.path.insert(0, '/game'); from browser_engine import browser_episode");return p;
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
async function handle(data){
 const portrait=data.payload.task==='portraits';
 const native=portrait||data.payload.backend==='mujoco-wasm';
 try{
  const p=await ready;
  if(native){if(!portrait)postMessage({type:'progress',message:'MUJOCO WASM · LOADING G1 / CHECKING RAY API'});await prepareNative(p);if(!portrait)postMessage({type:'progress',message:'MUJOCO WASM · COMPUTING G1 EPISODE'})}
  if(portrait)p.runPython('from portrait_poses import portrait_bundle');
  const fn=p.globals.get(portrait?'portrait_bundle':'browser_episode');
  try{
   const value=JSON.parse(fn(portrait?data.payload.reaction:JSON.stringify(data.payload)));
   const transfer=[];
   if(portrait&&data.payload.reaction==='normal'){
    value.renderAssets=wasmBridge.renderAssets();
    for(const mesh of value.renderAssets.meshes)transfer.push(mesh.positions.buffer,mesh.indices.buffer);
    if(value.still.length!==value.renderAssets.poseLength||Object.values(value.clips).some(clip=>clip.length!==60||clip.some(f=>f.length!==value.renderAssets.poseLength)))throw Error('Portrait dimensions mismatch');
   }
   if(!portrait&&native&&data.payload.render_3d&&!value.renderError){
    try{
     value.renderAssets=wasmBridge.renderAssets();
     for(const mesh of value.renderAssets.meshes)transfer.push(mesh.positions.buffer,mesh.indices.buffer);
     if(value.frames.some(f=>f.render_pose?.length!==value.renderAssets.poseLength))throw Error('Render pose shape mismatch');
    }catch(error){delete value.renderAssets;transfer.length=0;value.renderError=String(error)}
   }
   postMessage({id:data.id,value},transfer);
  }
  finally{fn.destroy()}
 }catch(e){postMessage({id:data.id,error:String(e)})}
 finally{if(native)wasmBridge.close()}
}
// The bridge owns a single compiled scene. Portrait and game jobs must run serially.
let queue=Promise.resolve();self.onmessage=({data})=>{queue=queue.then(()=>handle(data)).catch(error=>postMessage({id:data.id,error:String(error)}))};
