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
 function create(xml){close();drawingIds=[];if(!cachedAssets)throw Error('MuJoCo assets not prepared');
  const vfs=new mj.MjVFS();
  try{
   for(const [name,bytes] of cachedAssets)vfs.addBuffer('assets/'+name,bytes);
   const local=xml.replace(/meshdir="[^"]*"/,'meshdir="assets"');
   model=mj.from_xml_string(local,vfs);data=new mj.MjData(model);
  }catch(e){close();throw e}finally{vfs.delete()}
  const fields=['jnt_qposadr','jnt_dofadr','body_mocapid','body_parentid','geom_bodyid','geom_contype','geom_type','geom_size','geom_rbound','geom_group','geom_dataid','jnt_limited','jnt_range','mesh_vertadr','mesh_vertnum'];
  const info={ngeom:model.ngeom,njnt:model.njnt,nv:model.nv};
  for(const f of fields)info[f]=arr(model[f]);
  return toJson(info);
 }
 function constants(){const obj={},geom={};for(const n of ['BODY','JOINT','KEY','GEOM','SITE','MESH'])obj['mjOBJ_'+n]=numeric(mj.mjtObj['mjOBJ_'+n]);for(const n of ['BOX','SPHERE','CAPSULE','CYLINDER','ELLIPSOID','MESH'])geom['mjGEOM_'+n]=numeric(mj.mjtGeom['mjGEOM_'+n]);return toJson({obj,geom})}
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
 // Presentation-only snapshots from the compiled model. No mj_forward or physics calls here.
 let drawingIds=[];
 function drawingGeoms(){
  check();const out=[],meshType=numeric(mj.mjtGeom.mjGEOM_MESH),boxType=numeric(mj.mjtGeom.mjGEOM_BOX),capType=numeric(mj.mjtGeom.mjGEOM_CAPSULE),ellipsoidType=numeric(mj.mjtGeom.mjGEOM_ELLIPSOID);
  for(let i=0;i<model.ngeom;i++){
   const group=model.geom_group[i],type=model.geom_type[i];
   if(group===2&&(type===meshType||type===ellipsoidType)||group===5&&(type===boxType||type===capType))out.push(i);
  }
  return out;
 }
 function poseFor(id,source=data){return [...source.geom_xpos.slice(id*3,id*3+3),...source.geom_xmat.slice(id*9,id*9+9)]}
 function renderPose(){
  check();
  if(!drawingIds.length)drawingIds=drawingGeoms().filter(i=>model.geom_bodyid[i]!==0);
  return toJson(drawingIds.flatMap(id=>poseFor(id,data)));
 }
 function renderAssets(){
  check();const ids=drawingGeoms(),meshes=[],meshIndex=new Map(),geoms=[];
  drawingIds=ids.filter(i=>model.geom_bodyid[i]!==0);
  const meshType=numeric(mj.mjtGeom.mjGEOM_MESH),boxType=numeric(mj.mjtGeom.mjGEOM_BOX),ellipsoidType=numeric(mj.mjtGeom.mjGEOM_ELLIPSOID);
  const handNames=new Map();for(const side of ['left','right'])for(const kind of ['open','fist']){const name=side+'_'+kind+'_showcase',id=mj.mj_name2id(model,numeric(mj.mjtObj.mjOBJ_GEOM),name);if(id>=0)handNames.set(id,kind)}
  for(const id of ids){
   const type=model.geom_type[id],mid=model.geom_dataid[id],mat=model.geom_matid[id],group=model.geom_group[id];
   const moving=model.geom_bodyid[id]!==0;
   const rgba=arr(mat>=0?model.mat_rgba.slice(mat*4,mat*4+4):model.geom_rgba.slice(id*4,id*4+4));
   const g={id,kind:type===meshType?'mesh':type===boxType?'box':type===ellipsoidType?'ellipsoid':'capsule',hand:handNames.get(id)||null,group,rgba,size:arr(model.geom_size.slice(id*3,id*3+3)),moving,poseIndex:moving?drawingIds.indexOf(id):-1,pose:moving?null:poseFor(id)};
   if(g.kind==='mesh'){
    if(!meshIndex.has(mid)){
     const va=model.mesh_vertadr[mid],vn=model.mesh_vertnum[mid],fa=model.mesh_faceadr[mid],fn=model.mesh_facenum[mid];
     const positions=new Float32Array(model.mesh_vert.slice(va*3,(va+vn)*3)),indices=new Uint32Array(model.mesh_face.slice(fa*3,(fa+fn)*3));
     if(!positions.length||!indices.length)throw Error('Empty visual mesh '+mid);
     if(indices.some(k=>k>=vn))throw Error('Unexpected mesh face indices '+mid);
     meshIndex.set(mid,meshes.length);meshes.push({positions,indices});
    }
    g.mesh=meshIndex.get(mid);
   }
   geoms.push(g);
  }
  if(!meshes.length||!drawingIds.length)throw Error('No G1 visual geometry');
  return {meshes,geoms,poseLength:drawingIds.length*12};
 }
 // A separate MjData is used exclusively for walking presentation. Never modify evaluator data.
 function walkPose(phase,moved){
  check();if(!moved)return renderPose();
  if(!drawingIds.length)drawingIds=drawingGeoms().filter(i=>model.geom_bodyid[i]!==0);
  const visual=new mj.MjData(model);
  try{
   visual.qpos.set(data.qpos);visual.qvel.set(data.qvel);visual.mocap_pos.set(data.mocap_pos);visual.mocap_quat.set(data.mocap_quat);
   for(const [side,sign] of [['left',1],['right',-1]])for(const [joint,a] of [['hip_pitch',.18],['knee',.10],['shoulder_pitch',-.15]]){
    const id=mj.mj_name2id(model,numeric(mj.mjtObj.mjOBJ_JOINT),side+'_'+joint+'_joint');
    if(id>=0)visual.qpos[model.jnt_qposadr[id]]+=sign*a*Math.sin(phase*1.7);
   }
   mj.mj_forward(model,visual);
   return toJson(drawingIds.flatMap(id=>poseFor(id,visual)));
  }finally{visual.delete()}
 }
 function meshVertices(start,end){check();return toJson(arr(model.mesh_vert.slice(start*3,end*3)))}
 function jac(point,body){
  const a=new mj.DoubleBuffer(3*model.nv),b=new mj.DoubleBuffer(3*model.nv);
  try{mj.mj_jac(model,data,a,b,fromJson(point),body);return toJson([arr(view(a)),arr(view(b))])}
  finally{a.delete();b.delete()}
 }
 function matToQuat(values){const a=new mj.DoubleBuffer(4);try{mj.mju_mat2Quat(a,fromJson(values));return toJson(arr(view(a)))}finally{a.delete()}}
 function quatToVel(values,dt){const a=new mj.DoubleBuffer(3);try{mj.mju_quat2Vel(a,fromJson(values),dt);return toJson(arr(view(a)))}finally{a.delete()}}
 self.wasmBridge={prepare,create,constants,reset,forward,field,contacts,rays,close,renderPose,renderAssets,walkPose,meshVertices,jac,matToQuat,quatToVel,
   name2id:(type,name)=>mj.mj_name2id(model,type,name),id2name:(type,id)=>mj.mj_id2name(model,type,id)};
})();
