/* Actual G1 character visualization; independent of the game physics renderer. */
'use strict';
window.portraitView=(()=>{
 const CDN='https://cdn.jsdelivr.net/npm/three@0.178.0/build/';
 const palette={periodic:[.58,.81,1],age:[1,.76,.34],uncertainty:[.76,.55,1]};
 let T,renderer,scene,camera,bundle,normalPromise,defeatPromise,parts=[],stills={};
 function apply(o,p,i){const x=p[i],y=p[i+1],z=p[i+2];i+=3;
  o.matrix.set(p[i],p[i+1],p[i+2],x,p[i+3],p[i+4],p[i+5],y,p[i+6],p[i+7],p[i+8],z,0,0,0,1);
  o.matrixAutoUpdate=false;o.matrixWorldNeedsUpdate=true;
 }
 function build(data){
  T??=null;
  renderer=new T.WebGLRenderer({antialias:true,alpha:true,preserveDrawingBuffer:true});
  renderer.setPixelRatio(1);renderer.setSize(360,480,false);renderer.setClearColor(0,0);renderer.outputColorSpace=T.SRGBColorSpace;
  scene=new T.Scene();camera=new T.PerspectiveCamera(43,360/480,.01,20);camera.up.set(0,0,1);
  scene.add(new T.HemisphereLight(0xf4f8ff,0x78889e,2.2));
  for(const [x,y,z,intensity] of [[3,-4,5,2.4],[3,4,3,1.4],[-3,0,4,1.6]]){const l=new T.DirectionalLight(0xffffff,intensity);l.position.set(x,y,z);scene.add(l)}
  const meshes=data.meshes.map(m=>{const g=new T.BufferGeometry();g.setAttribute('position',new T.BufferAttribute(m.positions,3));g.setIndex(new T.BufferAttribute(m.indices,1));g.computeVertexNormals();return g});
  parts=data.geoms.map(g=>{
   const geometry=g.kind==='mesh'?meshes[g.mesh]:g.kind==='ellipsoid'?new T.SphereGeometry(1,20,16).scale(...g.size):null;
   if(!geometry||!g.moving)throw Error('Unexpected portrait geometry');
   const material=new T.MeshStandardMaterial({roughness:.53,metalness:.18,side:T.DoubleSide});
   const o=new T.Mesh(geometry,material);scene.add(o);return {o,g};
  });
 }
 function drawPose(name,pose,canvas,{bow=null,punch=false}={}){
  if(!renderer||pose?.length!==bundle.renderAssets.poseLength)return false;
  const tint=palette[name]||palette.periodic;
  for(const {o,g} of parts){
   const rgb=g.rgba.slice(0,3);
   const color=rgb.reduce((s,v)=>s+v,0)/3<.35?rgb.map(v=>Math.max(v,.2)):rgb.map((v,i)=>Math.min(1,v*.30+tint[i]*.70));
   o.material.color.setRGB(...color);
   o.visible=g.hand==='fist'?punch:g.hand==='open'?!punch:true;
   apply(o,pose,g.poseIndex*12);
  }
  const b=bow===null?0:bow,az=(bow===null?150:145)*Math.PI/180,elev=(bow===null?-8:-16-3*b)*Math.PI/180;
  const distance=bow===null?2.65:2.25-.4*b,target=new T.Vector3(bow===null?-.04:.12*b,0,bow===null?.83:.62-.30*b);
  camera.position.set(target.x-distance*Math.cos(az)*Math.cos(elev),target.y-distance*Math.sin(az)*Math.cos(elev),target.z-distance*Math.sin(elev));camera.lookAt(target);
  renderer.render(scene,camera);
  if(canvas){const ctx=canvas.getContext('2d');ctx.clearRect(0,0,canvas.width,canvas.height);ctx.drawImage(renderer.domElement,0,0,canvas.width,canvas.height)}
  return true;
 }
 function load(){
  normalPromise??=(async()=>{
   try{
    [T,bundle]=await Promise.all([import(CDN+'three.module.js'),browserGame.portraits()]);
    build(bundle.renderAssets);
    for(const name of Object.keys(palette)){drawPose(name,bundle.still,null);stills[name]=renderer.domElement.toDataURL('image/png')}
    return true;
   }catch(error){console.error('G1 portrait preparation failed',error);const el=document.getElementById('portraitStatus');el.textContent='G1 animation unavailable';el.title=String(error);el.hidden=false;return false}
  })();
  return normalPromise;
 }
 async function defeat(){
  if(!await load())return null;
  defeatPromise??=browserGame.portraits('defeat').then(data=>{
   if(data.frames.length!==54||data.bows.length!==54||data.frames.some(f=>f.length!==bundle.renderAssets.poseLength))throw Error('Defeat pose shape mismatch');
   return data;
  }).catch(error=>{console.error('G1 defeat preparation failed',error);toast('Defeat animation unavailable: '+error.message);return null});
  return defeatPromise;
 }
 function draw(name,index,canvas,loss=null){
  try{return loss?drawPose(name,loss.frames[index],canvas,{bow:loss.bows[index]}):drawPose(name,bundle.clips[name][index],canvas,{punch:name==='uncertainty'})}
  catch(error){console.warn('G1 portrait drawing failed',error);return false}
 }
 return {load,defeat,still:name=>stills[name],draw};
})();
