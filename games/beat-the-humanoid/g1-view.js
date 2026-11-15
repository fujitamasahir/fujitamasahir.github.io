/* Presentation-only Three.js viewer for recorded MuJoCo geometry and poses. */
'use strict';
window.g1View=(()=>{
 const CDN='https://cdn.jsdelivr.net/npm/three@0.178.0/build/';
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
 function clear(){ready=false;broken='';assets=null;resetScene()}
 async function prepare(data){
  clear();
  if(!data)return false;
  try{
   T??=await import(CDN+'three.module.js');
   if(!renderer){
    renderer=new T.WebGLRenderer({antialias:true,alpha:false,powerPreference:'high-performance'});
    renderer.setPixelRatio(Math.min(devicePixelRatio||1,2));
    renderer.setClearColor(0x101a26,1);
    renderer.outputColorSpace=T.SRGBColorSpace;
    renderer.domElement.className='g1-three-view';
    renderer.domElement.setAttribute('aria-label','Three-dimensional rendering of recorded MuJoCo G1 poses');
    renderer.domElement.addEventListener('webglcontextlost',event=>{event.preventDefault();broken='WebGL context lost';ready=false;renderer.domElement.hidden=true;document.querySelectorAll('#truthCanvas,#reviewTruth').forEach(x=>x.hidden=false);const el=document.getElementById('renderStatus');if(el)el.textContent='3D unavailable · 2D fallback'});
    renderer.domElement.addEventListener('webglcontextrestored',()=>{clear()});
   }
   if(!data.meshes.length||!data.geoms.length)throw Error('Empty geometry');
   assets=data;ready=true;return true;
  }catch(error){broken=String(error);console.warn('3D renderer unavailable',error);return false}
 }
 function geo(g){geometryCache.add(g);return g}
 function mat(options){const m=new T.MeshStandardMaterial(options);materialCache.add(m);return m}
 function mesh(g,m){const o=new T.Mesh(g,m);scene.add(o);return o}
 function applyPose(o,p,offset=0){
  const x=p[offset],y=p[offset+1],z=p[offset+2],i=offset+3;
  o.matrix.set(p[i],p[i+1],p[i+2],x,p[i+3],p[i+4],p[i+5],y,p[i+6],p[i+7],p[i+8],z,0,0,0,1);
  o.matrixAutoUpdate=false;o.matrixWorldNeedsUpdate=true;
 }
 function build(maze,strategy){
  resetScene();
  scene=new T.Scene();scene.background=new T.Color(0x101a26);
  scene.add(new T.HemisphereLight(0xe9f5ff,0x4b5365,2.4));
  const light=new T.DirectionalLight(0xfff4de,3.0);light.position.set(-6,-8,16);scene.add(light);
  const rim=new T.DirectionalLight(0x83bfff,1.1);rim.position.set(8,6,9);scene.add(rim);
  const w=maze.width*.75,h=maze.height*.75;
  const floor=mesh(geo(new T.PlaneGeometry(w,h)),mat({color:0x26394c,roughness:.95}));floor.position.z=-.02;
  const lines=[];
  for(let x=0;x<=maze.width;x++){const X=x*.75-w/2;lines.push(X,-h/2,.005,X,h/2,.005)}
  for(let y=0;y<=maze.height;y++){const Y=y*.75-h/2;lines.push(-w/2,Y,.005,w/2,Y,.005)}
  const lineGeo=geo(new T.BufferGeometry());lineGeo.setAttribute('position',new T.Float32BufferAttribute(lines,3));
  const lineMat=new T.LineBasicMaterial({color:0x61758a,transparent:true,opacity:.3});materialCache.add(lineMat);scene.add(new T.LineSegments(lineGeo,lineMat));
  const wallColor=({R:0x355c88,BP:0x327d70,M:0x76529b})[maze.mechanism]||0x355c88;
  const meshes=assets.meshes.map(m=>{const g=geo(new T.BufferGeometry());g.setAttribute('position',new T.BufferAttribute(m.positions,3));g.setIndex(new T.BufferAttribute(m.indices,1));g.computeVertexNormals();g.computeBoundingSphere();return g});
  const accent=({periodic:0x61b3ff,age:0xffbf4d,uncertainty:0xbe8bff})[strategy]||0x61b3ff;
  for(const g of assets.geoms){
   const geometry=g.kind==='mesh'?meshes[g.mesh]:g.kind==='box'?geo(new T.BoxGeometry(...g.size.map(x=>2*x))):geo(new T.CapsuleGeometry(g.size[0],2*g.size[1],8,16).rotateX(Math.PI/2));
   const color=g.group===2?new T.Color().setRGB(...g.rgba.slice(0,3)):g.kind==='capsule'?0xe95c62:g.moving?0xe59b4a:wallColor;
   const m=mat({color,roughness:g.group===2?.5:.85,metalness:g.group===2?.22:0,side:T.DoubleSide});
   if(g.group===2){m.emissive.setHex(accent);m.emissiveIntensity=.045;robotMaterials.push(m)}
   const o=mesh(geometry,m);
   if(g.moving)moving[g.poseIndex]=o;else applyPose(o,g.pose);
  }
  if(moving.length*12!==assets.poseLength||moving.some(x=>!x))throw Error('Incomplete dynamic geometry');
  const marker=(cell,color)=>{
   const ring=mesh(geo(new T.RingGeometry(.21,.31,40)),mat({color,emissive:color,emissiveIntensity:.45,side:T.DoubleSide}));
   ring.position.set((cell[0]-(maze.width-1)/2)*.75,(cell[1]-(maze.height-1)/2)*.75,.025);
  };
  marker(maze.start,0x6ab9ff);marker(maze.goal,0xffce63);
  const d=Math.hypot(w,h);camera=new T.OrthographicCamera(-1,1,1,-1,.05,d*5+20);
  camera.up.set(0,0,1);camera.position.set(0,-d*.62,d*1.55);camera.lookAt(0,0,.45);
  mazeId=maze.id;
 }
 function resize(){
  if(!renderer||!camera||!lastMaze)return;
  const el=renderer.domElement,w=el.clientWidth,h=el.clientHeight;
  if(!w||!h)return;
  renderer.setSize(w,h,false);
  const aspect=w/h,halfW=lastMaze.width*.75/2+.85,halfH=lastMaze.height*.75/2+1.25;
  const extent=Math.max(halfH,halfW/aspect);
  camera.left=-extent*aspect;camera.right=extent*aspect;camera.top=extent;camera.bottom=-extent;camera.updateProjectionMatrix();
  renderer.render(scene,camera);
 }
 function draw(canvas,maze,frame){
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
 return {prepare,draw,hide,clear,error:()=>broken};
})();
