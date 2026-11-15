/* Official MuJoCo 3.17.0, single-thread WASM. Only the worker owns these handles. */
(() => {
 const CDN='https://cdn.jsdelivr.net/npm/@mujoco/mujoco@3.17.0/';
 let modulePromise, assetsPromise, mj, model, data;
 const fromJson=JSON.parse, toJson=JSON.stringify, arr=x=>Array.from(x);
 async function checked(url,sha){
  const response=await fetch(url);
  if(!response.ok)throw Error('G1 asset HTTP '+response.status+': '+url);
  const bytes=new Uint8Array(await response.arrayBuffer());
  const actual=Array.from(new Uint8Array(await crypto.subtle.digest('SHA-256',bytes)),x=>x.toString(16).padStart(2,'0')).join('');
  if(actual!==sha)throw Error('G1 asset SHA-256 mismatch: '+url);
  return bytes;
 }
 async function load(){
  modulePromise??=(async()=>{const {default:loadMujoco}=await import(CDN+'mujoco.js');return loadMujoco({locateFile:name=>CDN+name})})();
  assetsPromise??=(async()=>{
   const manifestResponse=await fetch('./models/manifest.json');
   if(!manifestResponse.ok)throw Error('G1 manifest unavailable');
   const manifest=await manifestResponse.json();
   const source='https://raw.githubusercontent.com/'+manifest.source_repo+'/'+manifest.source_commit+'/third_party/unitree_g1/assets/';
   const results=new Map(),queue=manifest.meshes.slice();
   await Promise.all(Array.from({length:5},async()=>{while(queue.length){const item=queue.shift(),bytes=await checked(source+encodeURIComponent(item.name),item.sha256);results.set(item.name,bytes)}}));
   return results;
  })();
  mj=await modulePromise;await assetsPromise;
 }
 function close(){try{data?.delete()}finally{data=null;try{model?.delete()}finally{model=null}}}
 function check(){if(!model||!data)throw Error('MuJoCo scene not initialized')}
 function numeric(e){return Number(e?.value??e)}
 function view(b){return typeof b.GetView==='function'?b.GetView():b.getView()}
 let cachedAssets=null;
 async function prepare(){await load();cachedAssets=await assetsPromise;
  const probe=mj.from_xml_string('<mujoco><worldbody><geom name="target" type="sphere" size=".1" pos="1 0 0"/></worldbody></mujoco>');
  const d=new mj.MjData(probe),hit=new mj.IntBuffer(1);try{
   mj.mj_forward(probe,d);
   const distance=mj.mj_ray(probe,d,[0,0,0],[1,0,0],null,true,-1,hit,null);
   if(Math.abs(distance-.9)>1e-5||view(hit)[0]!==0)throw Error('MuJoCo ray self-test failed: '+distance);
  }finally{hit.delete();d.delete();probe.delete()}
  return true;
 }
 function create(xml){close();if(!cachedAssets)throw Error('MuJoCo assets not prepared');
  const vfs=new mj.MjVFS();
  try{
   for(const [name,bytes] of cachedAssets)vfs.addBuffer('assets/'+name,bytes);
   const local=xml.replace(/meshdir="[^"]*"/,'meshdir="assets"');
   model=mj.from_xml_string(local,vfs);data=new mj.MjData(model);
  }catch(e){close();throw e}finally{vfs.delete()}
  const fields=['jnt_qposadr','jnt_dofadr','body_mocapid','body_parentid','geom_bodyid','geom_contype','geom_type','geom_size','geom_rbound'];
  const info={ngeom:model.ngeom};
  for(const f of fields)info[f]=arr(model[f]);
  return toJson(info);
 }
 function constants(){const obj={},geom={};for(const n of ['BODY','JOINT','KEY','GEOM','SITE'])obj['mjOBJ_'+n]=numeric(mj.mjtObj['mjOBJ_'+n]);for(const n of ['BOX','SPHERE','CAPSULE','CYLINDER','ELLIPSOID'])geom['mjGEOM_'+n]=numeric(mj.mjtGeom['mjGEOM_'+n]);return toJson({obj,geom})}
 function reset(key){check();mj.mj_resetDataKeyframe(model,data,key);return toJson({qpos:arr(data.qpos),qvel:arr(data.qvel),mocap_pos:arr(data.mocap_pos)})}
 function forward(payload){check();const p=fromJson(payload);data.qpos.set(p.qpos);data.qvel.set(p.qvel);data.mocap_pos.set(p.mocap_pos);mj.mj_forward(model,data)}
 function field(name){check();return toJson(arr(data[name]))}
 function contacts(){check();const pairs=[],vector=data.contact;try{for(let i=0;i<data.ncon;i++){const v=vector.get(i);try{pairs.push([v.geom1,v.geom2])}finally{v?.delete?.()}}}finally{vector?.delete?.()}return toJson(pairs)}
 function rays(payload){
  check();const p=fromJson(payload),n=p.directions.length/3;
  const ids=new mj.IntBuffer(n),dist=new mj.DoubleBuffer(n);
  try{
   view(ids).fill(-1);view(dist).fill(-1);
   if(p.multi)mj.mj_multiRay(model,data,p.origin,p.directions,p.groups,p.static,p.exclude,ids,dist,null,n,p.cutoff);
   else for(let i=0;i<n;i++){
    const one=new mj.IntBuffer(1);try{view(one)[0]=-1;const d=mj.mj_ray(model,data,p.origin,p.directions.slice(3*i,3*i+3),p.groups,p.static,p.exclude,one,null);view(ids)[i]=view(one)[0];view(dist)[i]=d}finally{one.delete()}
   }
   return toJson({ids:arr(view(ids)),dist:arr(view(dist))});
  }finally{ids.delete();dist.delete()}
 }
 self.wasmBridge={prepare,create,constants,reset,forward,field,contacts,rays,close,
   name2id:(type,name)=>mj.mj_name2id(model,type,name),id2name:(type,id)=>mj.mj_id2name(model,type,id)};
})();
