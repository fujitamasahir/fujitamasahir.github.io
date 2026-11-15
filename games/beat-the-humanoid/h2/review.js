/* Viewer for the archived H2 high-frequency experiment; all displayed states are recorded. */
'use strict';
const $=id=>document.getElementById(id);
let index,entry,record,cursor=0,playing=false,timer=null,serial=0,download=null;
const cache=new Map();
const selectIds=['map','schedule','model'];
const eventLabels={initial_state:'INITIAL STATE',addition:'OBJECT APPEARED',relocation:'OBJECT MOVED',removal:'OBJECT DISAPPEARED'};
const coords=c=>Array.isArray(c)?'('+c.join(', ')+')':'—';
const labels={move:'FORWARD / MOVE',sidestep:'SIDESTEP',retreat:'RETREAT',turn:'TURN',wait:'WAIT'};
function options(id,rows){const el=$(id);el.replaceChildren();for(const [value,label] of rows){const option=document.createElement('option');option.value=value;option.textContent=label;el.append(option)}}
function showStatus(message,error=false,retry=false){const el=$('status');el.hidden=false;el.classList.toggle('error',error);el.replaceChildren(document.createTextNode(message));if(retry){const b=document.createElement('button');b.textContent='RETRY';b.onclick=loadSelection;el.append(b)}}
function pause(){playing=false;clearTimeout(timer);timer=null;$('play').textContent='▶ PLAY';$('play').setAttribute('aria-pressed','false')}
function selectEntry(){return H2Replay.getEntry(index,$('map').value,$('schedule').value,$('model').value)}
async function loadSelection(){
 if(!index)return;
 const id=++serial;download?.abort();download=new AbortController();pause();record=null;
 $('review').hidden=true;$('verified').hidden=true;
 entry=selectEntry();const e=entry;
 $('modelGoals').textContent=e.model.goals+' / '+e.model.episodes;
 const params=new URLSearchParams({map:String(e.map.number),schedule:String(e.schedule),model:e.model.id});history.replaceState(null,'','?'+params.toString());
 showStatus('LOADING RECORDED EPISODE…');
 try{
  let data=cache.get(e.filename);
  if(!data){data=await H2Replay.loadReplay(index,e,download.signal);if(id!==serial)return;cache.set(e.filename,data);if(cache.size>5)cache.delete(cache.keys().next().value)}
  if(id!==serial)return;
  record=data;cursor=0;$('verified').hidden=false;$('status').hidden=true;$('review').hidden=false;
  $('outcome').textContent=e.success?'GOAL REACHED':'NOT REACHED';$('outcome').className=e.success?'success':'fail';
  $('finalSteps').textContent=e.steps+' / '+e.stepLimit;$('finalScans').textContent=e.paidScans+' / '+e.scanLimit;
  $('distance').textContent=e.distanceM.toFixed(2)+' m';$('contacts').textContent=e.unwantedContacts;
  $('original').href='https://github.com/'+index.sourceRepo+'/blob/'+index.sourceCommit+'/'+index.sourcePath+'replays/primary/'+e.filename;
  $('slider').max=String(record.frames.length-1);render(0);
 }catch(error){if(id===serial&&error.name!=='AbortError')showStatus(error.message,true,true)}
}
const colors={free:'#152330',unknown:'#485364',memoryFree:'#174247',memoryOccupied:'#9a6250',object:'#f19a43',ped:'#f26970',robot:'#64b4ff',goal:'#ffd074',event:'#fff2bb'};
function draw(canvas,frame,isMemory){
 const ctx=canvas.getContext('2d'),r=record.replay,w=r.grid.width,h=r.grid.height;
 const inset=9,cell=Math.min((canvas.width-2*inset)/w,(canvas.height-2*inset)/h),x0=(canvas.width-w*cell)/2,y0=(canvas.height-h*cell)/2;
 const point=c=>[x0+(c[0]+.5)*cell,y0+(h-c[1]-.5)*cell];
 const box=c=>[x0+c[0]*cell,y0+(h-1-c[1])*cell];
 function fill(c,color,pad=1){const [x,y]=box(c);ctx.fillStyle=color;ctx.fillRect(x+pad,y+pad,cell-2*pad,cell-2*pad)}
 function ring(c,color,rad=.34){const [x,y]=point(c);ctx.beginPath();ctx.arc(x,y,cell*rad,0,Math.PI*2);ctx.strokeStyle=color;ctx.lineWidth=2.5;ctx.stroke()}
 function line(path,color,width,dashes=[]){if(path.length<2)return;ctx.beginPath();path.forEach((c,i)=>{const p=point(c);i?ctx.lineTo(...p):ctx.moveTo(...p)});ctx.setLineDash(dashes);ctx.strokeStyle=color;ctx.lineWidth=width;ctx.lineJoin='round';ctx.lineCap='round';ctx.stroke();ctx.setLineDash([])}
 ctx.fillStyle='#0b111a';ctx.fillRect(0,0,canvas.width,canvas.height);
 const wall=({R:'#355c88',BP:'#327d70',M:'#76529b'})[r.map_family]||'#355c88';
 for(let y=0;y<h;y++)for(let x=0;x<w;x++){
  const i=y*w+x,c=[x,y],v=frame.memory[i];
  fill(c,isMemory?(v===-1?colors.unknown:v===0?colors.memoryFree:colors.memoryOccupied):record.walls.has(c.join(','))?wall:frame.truth[i]?colors.object:colors.free,.65);
 }
 if(isMemory&&frame.observation_granted){
  ctx.strokeStyle='#53dceab0';ctx.lineWidth=1.5;
  for(const cellData of frame.observed_cells_xy_value||[]){const [x,y]=box(cellData);ctx.strokeRect(x+2,y+2,cell-4,cell-4)}
 }
 if(!isMemory){
  for(const o of frame.objects_after){fill(o.cell,colors.object,cell*.18);const [x,y]=point(o.cell);ctx.fillStyle='#8a4b1b';ctx.fillRect(x-cell*.15,y-cell*.15,cell*.3,cell*.3)}
  for(const ped of frame.pedestrians_xy){const [x,y]=point(ped.cell);ctx.beginPath();ctx.arc(x,y,cell*.28,0,Math.PI*2);ctx.fillStyle=colors.ped;ctx.fill();ctx.strokeStyle='#ffc3c3';ctx.lineWidth=1.8;ctx.stroke()}
  const ev=frame.event,from=ev.from_cell,to=ev.to_cell;
  if(from){ctx.setLineDash([4,3]);ring(from,colors.event,.43);ctx.setLineDash([])}
  if(to)ring(to,colors.event,.43);
  if(from&&to){line([from,to],colors.event,1.5,[5,4]);const [x,y]=point(to),[fx,fy]=point(from),a=Math.atan2(y-fy,x-fx);ctx.strokeStyle=colors.event;ctx.lineWidth=2;ctx.beginPath();ctx.moveTo(x-8*Math.cos(a-.45),y-8*Math.sin(a-.45));ctx.lineTo(x,y);ctx.lineTo(x-8*Math.cos(a+.45),y-8*Math.sin(a+.45));ctx.stroke()}
 }
 const trajectory=record.frames.slice(0,cursor+1).map(f=>f.robot_position_after_action_xy);
 line(trajectory,'#64b4ffaa',Math.max(2,cell*.09));
 ring(r.start_xy,'#9ec8e5',.30);ring(r.goal_xy,colors.goal,.38);
 const [gx,gy]=point(r.goal_xy);ctx.fillStyle=colors.goal;ctx.beginPath();ctx.arc(gx,gy,cell*.15,0,Math.PI*2);ctx.fill();
 const [rx,ry]=point(frame.robot_position_after_action_xy);ctx.save();ctx.translate(rx,ry);ctx.rotate(-frame.robot_heading_after_action_rad);ctx.fillStyle=colors.robot;ctx.strokeStyle='#c9ebff';ctx.lineWidth=1.5;ctx.beginPath();ctx.moveTo(cell*.38,0);ctx.lineTo(-cell*.25,-cell*.23);ctx.lineTo(-cell*.18,0);ctx.lineTo(-cell*.25,cell*.23);ctx.closePath();ctx.fill();ctx.stroke();ctx.restore();
}
function render(value){
 if(!record)return;cursor=Math.max(0,Math.min(record.frames.length-1,Number(value)||0));
 const f=record.frames[cursor],ev=f.event;
 $('slider').value=String(cursor);$('first').disabled=$('prev').disabled=cursor===0;$('next').disabled=$('last').disabled=cursor===record.frames.length-1;
 $('stepCount').textContent=f.step+' / '+entry.steps;
 $('scanCount').textContent=f.paid_observation_count+' / '+entry.scanLimit;
 $('scanState').textContent=f.observation_granted?'PAID SCAN GRANTED':f.observation_requested?'SCAN REQUESTED · NOT GRANTED':'NO PAID SCAN';
 $('eventType').textContent=eventLabels[ev.event_type]||ev.event_type;
 $('eventDetail').textContent=ev.event_type==='initial_state'?'Initial objects: '+f.objects_after.length:[ev.object_id,ev.from_cell?'FROM '+coords(ev.from_cell):'',ev.to_cell?'TO '+coords(ev.to_cell):'','Objects: '+ev.object_count_before+' → '+ev.object_count_after].filter(Boolean).join(' · ');
 const action=f.executed_action;
 $('action').textContent=Array.isArray(action)?(labels[action[0]]||String(action[0]).toUpperCase()):(f.step===0?'INITIAL POSE':String(action??'NO ACTION').toUpperCase());
 const rejection=f.selected_action_shield_rejection,intervention=f.emergency_safety_intervention;
 $('safety').textContent=[Array.isArray(action)&&action[0]!=='wait'?'Action: '+(Array.isArray(action[1])?coords(action[1]):String(action[1]??'')):'',intervention?'Safety: '+(intervention.reason||'intervention'):f.emergency_safety_scan?'Independent safety scan':'No safety scan',rejection?.rejected?'Shield rejected: '+(rejection.reason||'action'):''].filter(Boolean).join(' · ');
 const known=f.memory.reduce((s,v)=>s+(v!==-1?1:0),0);
 $('memoryDetail').textContent=known+' / '+f.memory.length+' cells known';
 const hits=Object.values(f.unwanted_contacts||{}).reduce((s,n)=>s+n,0);
 $('contactDetail').textContent='Unwanted contact pairs at this step: '+hits+(f.goal_reached?' · Goal reached':'');
 draw($('truth'),f,false);draw($('memory'),f,true);
}
function seek(value){pause();render(value)}
function startPlayback(){if(!record)return;if(cursor===record.frames.length-1)render(0);playing=true;$('play').textContent='Ⅱ PAUSE';$('play').setAttribute('aria-pressed','true');scheduleTick()}
function scheduleTick(){clearTimeout(timer);if(!playing)return;timer=setTimeout(()=>{if(!playing||!record)return;if(cursor>=record.frames.length-1){pause();return}render(cursor+1);if(cursor>=record.frames.length-1)pause();else scheduleTick()},200/Number($('speed').value))}
function toggle(){playing?pause():startPlayback()}
$('first').onclick=()=>seek(0);$('prev').onclick=()=>seek(cursor-1);$('next').onclick=()=>seek(cursor+1);$('last').onclick=()=>record&&seek(record.frames.length-1);
$('slider').oninput=e=>seek(e.target.value);$('play').onclick=toggle;$('speed').onchange=()=>{if(playing)scheduleTick()};
for(const id of selectIds)$(id).onchange=loadSelection;
window.addEventListener('keydown',e=>{if(['SELECT','INPUT','BUTTON','A'].includes(e.target.tagName))return;const k=e.key;
 if(k==='ArrowLeft'){e.preventDefault();seek(cursor-1)}else if(k==='ArrowRight'){e.preventDefault();seek(cursor+1)}else if(k==='Home'){e.preventDefault();seek(0)}else if(k==='End'){e.preventDefault();if(record)seek(record.frames.length-1)}else if(k===' '){e.preventDefault();if(!e.repeat)toggle()}else if(k==='Escape')location.href='./'});
let previous=new Set(),lastDir=0,nextRepeat=0;
function gamepad(now){const pad=[...(navigator.getGamepads?.()||[])].find(Boolean);if(pad){const buttons=new Set(pad.buttons.flatMap((b,i)=>b.pressed?[i]:[])),edge=i=>buttons.has(i)&&!previous.has(i);
 if(edge(1)){location.href='./';return}
 if(record){if(edge(0))toggle();else if(edge(6))seek(0);else if(edge(7))seek(record.frames.length-1);else{
 const axis=pad.axes[0]||0,d=buttons.has(14)?-1:buttons.has(15)?1:Math.abs(axis)>.4?Math.sign(axis):0;
 if(d&&(d!==lastDir||now>=nextRepeat)){seek(cursor+d);nextRepeat=now+(d!==lastDir?330:95)}lastDir=d
 }}previous=buttons}else{previous=new Set();lastDir=0}requestAnimationFrame(gamepad)}
(async()=>{try{
 const response=await fetch('./h2/index.json');if(!response.ok)throw Error('Experiment index unavailable');
 index=H2Replay.validateIndex(await response.json());
 options('map',index.maps.map(m=>[m.number,'Maze '+String(m.number).padStart(2,'0')+' · '+m.family]));
 options('model',index.arms.map(m=>[m.id,m.label]));
 const p=new URLSearchParams(location.search);
 if(index.maps.some(m=>String(m.number)===p.get('map')))$('map').value=p.get('map');
 if(['0','1'].includes(p.get('schedule')))$('schedule').value=p.get('schedule');
 if(index.arms.some(m=>m.id===p.get('model')))$('model').value=p.get('model');
 for(const id of selectIds)$(id).disabled=false;
 await loadSelection();requestAnimationFrame(gamepad)
 }catch(error){showStatus(error.message,true);const b=document.createElement('button');b.textContent='RELOAD';b.onclick=()=>location.reload();$('status').append(b)}})();
