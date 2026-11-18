/* Actual G1 character visualization; independent of the game physics renderer. */
'use strict';
window.portraitView=(()=>{
 const CDN='https://cdn.jsdelivr.net/npm/three@0.178.0/build/';
 const palette={periodic:[.58,.81,1],age:[1,.76,.34],uncertainty:[.76,.55,1]};
 let T,renderer,scene,camera,bundle,normalPromise,defeatPromise,parts=[],stills={},staticMode=false,staticClips={};
 const image=src=>new Promise((resolve,reject)=>{const im=new Image();im.onload=()=>resolve(im);im.onerror=()=>reject(Error('Portrait image unavailable: '+src));im.src=src});
 const base='./portraits/native/';
 async function loadNative(){
  const response=await fetch(base+'manifest.json',{cache:'no-cache'});
  if(!response.ok)return false;
  const manifest=await response.json();
  if(manifest.source_commit!=='b50f81255756a9c8bd55dddd2ea72c54d58ebb7d'||manifest.version!==1)throw Error('Unexpected native portrait manifest');
  const loaded=await Promise.all(Object.keys(palette).map(async name=>{
   const [still,clip]=await Promise.all([image(base+name+'.png'),image(base+name+'-motion.png')]);
   if(still.naturalWidth!==360||still.naturalHeight!==480||clip.naturalWidth!==2160||clip.naturalHeight!==4800)throw Error('Native portrait dimensions mismatch');
   return {name,still,clip};
  }));
  for(const {name,still,clip} of loaded){stills[name]=still.src;staticClips[name]=clip}
  staticMode=true;return true;
 }
 function drawSheet(sheet,index,canvas){
  const ctx=canvas.getContext('2d'),x=index%6*360,y=Math.floor(index/6)*480;
  ctx.clearRect(0,0,canvas.width,canvas.height);
  ctx.drawImage(sheet,x,y,360,480,0,0,canvas.width,canvas.height);return true;
 }
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
    try{if(await loadNative())return true}catch(error){console.warn('Native portraits unavailable, trying WASM',error)}
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
  if(staticMode){
   defeatPromise??=Promise.all(Object.keys(palette).map(async name=>{const im=await image(base+name+'-defeat-motion-v14.png');if(im.naturalWidth!==2160||im.naturalHeight!==4320)throw Error('Native defeat dimensions mismatch');return [name,im]})).then(pairs=>({native:true,sheets:Object.fromEntries(pairs)})).catch(error=>{console.error('Native defeat unavailable',error);toast(error.message);return null});
   return defeatPromise;
  }
  defeatPromise??=browserGame.portraits('defeat').then(data=>{
   if(data.frames.length!==54||data.bows.length!==54||data.frames.some(f=>f.length!==bundle.renderAssets.poseLength))throw Error('Defeat pose shape mismatch');
   return data;
  }).catch(error=>{console.error('G1 defeat preparation failed',error);toast('Defeat animation unavailable: '+error.message);return null});
  return defeatPromise;
 }
 function draw(name,index,canvas,loss=null){
  try{if(staticMode)return drawSheet(loss?loss.sheets[name]:staticClips[name],index,canvas);return loss?drawPose(name,loss.frames[index],canvas,{bow:loss.bows[index]}):drawPose(name,bundle.clips[name][index],canvas,{punch:name==='uncertainty'})}
  catch(error){console.error('G1 portrait drawing failed',error);toast('G1 animation unavailable: '+error.message);return false}
 }
 return {load,defeat,still:name=>stills[name],draw,mode:()=>staticMode?'native sprites':bundle?'WASM':'loading'};
})();
