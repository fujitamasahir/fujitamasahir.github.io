/* Presentation-only Three.js viewer for recorded MuJoCo geometry and poses. */
'use strict';
window.g1View=(()=>{
 const CDN='https://cdn.jsdelivr.net/npm/three@0.178.0/build/';
 const revisions=new Map();
 let activeRevision=null;
 let T,renderer,scene,camera,assets,mazeId,ready=false,broken='',moving=[],robotMaterials=[],observeResize,lastFrame,lastMaze;
 const geometryCache=new Set(),materialCache=new Set();
 function resetScene(){
  if(observeResize)observeResize.disconnect();
  for(const g of geometryCache)g.dispose();
  for(const m of materialCache)m.dispose();
  geometryCache.clear();materialCache.clear();
  scene=null;camera=null;moving=[];robotMaterials=[];mazeId=null;lastFrame=null;lastMaze=null;
  if(renderer)renderer.domElement.hidden=true;
 }
 function clear(){ready=false;broken='';assets=null;revisions.clear();activeRevision=null;resetScene()}
 async function prepare(data,revision=0){
  clear();
  if(!data)return false;
  try{
   T??=await import(CDN+'three.module.js');
   if(!renderer){
    renderer=new T.WebGLRenderer({antialias:true,alpha:false,powerPreference:'high-performance'});
    renderer.setPixelRatio(Math.min(devicePixelRatio||1,2));
    renderer.setClearColor(0x000000,1);
    renderer.outputColorSpace=T.SRGBColorSpace;
    renderer.domElement.className='g1-three-view';
    renderer.domElement.setAttribute('aria-label','Three-dimensional rendering of recorded MuJoCo G1 poses');
    renderer.domElement.addEventListener('webglcontextlost',event=>{event.preventDefault();broken='WebGL context lost';ready=false;renderer.domElement.hidden=true;document.querySelectorAll('#truthCanvas,#reviewTruth').forEach(x=>x.hidden=false);const el=document.getElementById('renderStatus');if(el)el.textContent='3D unavailable · 2D fallback'});
    renderer.domElement.addEventListener('webglcontextrestored',()=>{clear()});
   }
   if(!data.meshes.length||!data.geoms.length)throw Error('Empty geometry');
   assets=data;revisions.set(revision,data);activeRevision=revision;ready=true;return true;
  }catch(error){broken=String(error);console.warn('3D renderer unavailable',error);return false}
 }
 function geo(g){geometryCache.add(g);return g}
 function mat(options){const m=new T.MeshLambertMaterial(options);materialCache.add(m);return m}
 function mesh(g,m){const o=new T.Mesh(g,m);scene.add(o);return o}
 function applyPose(o,p,offset=0){
  const x=p[offset],y=p[offset+1],z=p[offset+2],i=offset+3;
  o.matrix.set(p[i],p[i+1],p[i+2],x,p[i+3],p[i+4],p[i+5],y,p[i+6],p[i+7],p[i+8],z,0,0,0,1);
  o.matrixAutoUpdate=false;o.matrixWorldNeedsUpdate=true;
 }
 function build(maze,strategy){
  resetScene();
  scene=new T.Scene();scene.background=new T.Color(0x000000);
  // Match the native ShowcaseRenderer: broad directional fill, no shadows/specular.
  scene.add(new T.AmbientLight(0xffffff,.76));
  const light=new T.DirectionalLight(0xffffff,.72);light.position.set(-.22,-.32,.92);scene.add(light);
  const w=maze.width*.75,h=maze.height*.75;
  const colorRGB=(r,g,b)=>new T.Color().setRGB(r,g,b,T.SRGBColorSpace);
  const floor=mesh(geo(new T.PlaneGeometry(2*Math.max(2,w/2+.75),2*Math.max(2,h/2+.75))),mat({color:colorRGB(.72,.76,.80),side:T.DoubleSide}));floor.position.z=0;
  const wallColor=colorRGB(.62,.68,.76);
  const meshes=assets.meshes.map(m=>{const g=geo(new T.BufferGeometry());g.setAttribute('position',new T.BufferAttribute(m.positions,3));g.setIndex(new T.BufferAttribute(m.indices,1));g.computeVertexNormals();g.computeBoundingSphere();return g});
  for(const g of assets.geoms){
   const geometry=g.kind==='mesh'?meshes[g.mesh]:g.kind==='box'?geo(new T.BoxGeometry(...g.size.map(x=>2*x))):geo(new T.CapsuleGeometry(g.size[0],2*g.size[1],8,16).rotateX(Math.PI/2));
   const color=g.group===2?colorRGB(...g.rgba.slice(0,3)):g.kind==='capsule'?colorRGB(.95,.24,.33):g.moving?colorRGB(.95,.52,.18):wallColor;
   const m=mat({color,side:T.DoubleSide});
   const o=mesh(geometry,m);
   if(g.moving)moving[g.poseIndex]=o;else applyPose(o,g.pose);
  }
  if(moving.length*12!==assets.poseLength||moving.some(x=>!x))throw Error('Incomplete dynamic geometry');
  // The source draws a gold cylinder and sphere at Goal, with no Start ring.
  const gx=(maze.goal[0]-(maze.width-1)/2)*.75,gy=(maze.goal[1]-(maze.height-1)/2)*.75;
  const goalBase=mesh(geo(new T.CylinderGeometry(.28,.28,.07,32).rotateX(Math.PI/2)),mat({color:colorRGB(1,.65,.06)}));goalBase.position.set(gx,gy,.04);
  const goalBall=mesh(geo(new T.SphereGeometry(.19,24,16)),mat({color:colorRGB(1,.8,.12)}));goalBall.position.set(gx,gy,1.1);
  // mjv_defaultCamera + ShowcaseRenderer's distance, azimuth=90°, elevation=-68°.
  const distance=Math.max(w,h)*1.65,elev=68*Math.PI/180;
  camera=new T.PerspectiveCamera(45,4/3,.05,distance*5+20);
  camera.up.set(0,0,1);camera.position.set(0,-distance*Math.cos(elev),.5+distance*Math.sin(elev));camera.lookAt(0,0,.5);
  mazeId=maze.id;
 }
 function resize(){
  if(!renderer||!camera||!lastMaze)return;
  const el=renderer.domElement,w=el.clientWidth,h=el.clientHeight;
  if(!w||!h)return;
  renderer.setSize(w,h,false);
  // Preserve the source 480×360 aspect; center the 4:3 perspective within the panel.
  const vw=Math.min(w,h*4/3),vh=vw*3/4;
  renderer.setScissorTest(false);renderer.setViewport(0,0,w,h);renderer.clear();
  renderer.setViewport((w-vw)/2,(h-vh)/2,vw,vh);renderer.render(scene,camera);
 }
 function register(revision,data){if(data?.meshes?.length&&data?.geoms?.length)revisions.set(revision,data)}
 function draw(canvas,maze,frame){
  const revision=frame.scene_revision??0;
  if(ready&&activeRevision!==revision){const next=revisions.get(revision);if(!next)return false;resetScene();assets=next;activeRevision=revision;}
  if(!ready||!assets||!Array.isArray(frame.render_pose)||frame.render_pose.length!==assets.poseLength)return false;
  try{
   if(!scene||mazeId!==maze.id)build(maze,frame.strategy);
   for(let i=0;i<moving.length;i++)applyPose(moving[i],frame.render_pose,i*12);
   lastMaze=maze;lastFrame=frame;
   const el=renderer.domElement;
   if(el.parentElement!==canvas.parentElement){
    observeResize?.disconnect();canvas.parentElement.insertBefore(el,canvas.nextSibling);
    observeResize=new ResizeObserver(resize);observeResize.observe(canvas.parentElement);
   }
   el.hidden=false;canvas.hidden=true;resize();return true;
  }catch(error){broken=String(error);console.warn('3D frame unavailable',error);ready=false;if(renderer)renderer.domElement.hidden=true;canvas.hidden=false;return false}
 }
 function hide(){if(renderer)renderer.domElement.hidden=true}
 return {prepare,register,draw,hide,clear,error:()=>broken};
})();
