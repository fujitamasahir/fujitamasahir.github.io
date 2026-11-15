/* Read-only H2 replay reader. No simulator, regenerated trajectory or inferred memory. */
'use strict';
(function(root){
 const fields=['sha256','bytes','success','steps','paidScans','unwantedContacts','distanceM','stepLimit','scanLimit'];
 const base64=value=>Uint8Array.from(atob(value),c=>c.charCodeAt(0));
 function decodeMemory(value,width,height){
  const bytes=base64(value),n=width*height;
  if(bytes.length!==Math.ceil(n/4))throw Error('Invalid spatial memory length');
  return Array.from({length:n},(_,i)=>{const v=(bytes[i>>2]>>(2*(i%4)))&3;if(v===3)throw Error('Invalid ternary memory value');return v-1});
 }
 function decodeBits(value,width,height){
  const bytes=base64(value),n=width*height;
  if(bytes.length!==Math.ceil(n/8))throw Error('Invalid occupancy length');
  return Array.from({length:n},(_,i)=>(bytes[i>>3]>>(i%8))&1);
 }
 function getEntry(index,mapNumber,schedule,arm){
  const map=index.maps.find(m=>m.number===Number(mapNumber)), model=index.arms.find(a=>a.id===arm);
  const tuple=map?.schedules?.[Number(schedule)]?.[arm];
  if(!map||!model||![0,1].includes(Number(schedule))||!Array.isArray(tuple)||tuple.length!==fields.length)throw Error('Unknown replay');
  const caseNumber=(map.number-1)*2+Number(schedule)+1;
  const filename='episode_'+String(caseNumber).padStart(3,'0')+'_'+model.id+'.json.gz';
  return {map,model,schedule:Number(schedule),filename,...Object.fromEntries(fields.map((f,i)=>[f,tuple[i]]))};
 }
 function validateIndex(index){
  if(index.version!==1||index.maps.length!==36||index.arms.length!==13||index.fields.join()!==fields.join())throw Error('Unexpected H2 manifest');
  let episodes=0,goals=0,steps=0,contacts=0;
  for(const map of index.maps)for(const schedule of [0,1])for(const arm of index.arms){
   const e=getEntry(index,map.number,schedule,arm.id);
   if(!/^[a-f0-9]{64}$/.test(e.sha256)||!Number.isInteger(e.steps)||e.steps<0||e.steps>e.stepLimit||e.paidScans>e.scanLimit)throw Error('Invalid indexed episode');
   episodes++;goals+=e.success;steps+=e.steps;contacts+=e.unwantedContacts;
  }
  if(episodes!==index.totals.episodes||goals!==index.totals.goals||steps!==index.totals.steps||contacts!==index.totals.contacts)throw Error('H2 index aggregate mismatch');
  return index;
 }
 function validateReplay(body,entry,index){
  const r=body.replay,w=r?.grid?.width,h=r?.grid?.height;
  if(r?.schema_version!=='1.0.0'||r.episode_key?.join('|')!==[entry.map.identity,entry.schedule,entry.model.id,index.seed].join('|')||w!==25||h!==25||r.map_family!==entry.map.family)throw Error('Episode identity or schema mismatch');
  if(!Array.isArray(r.steps)||r.steps.length!==entry.steps+1||Number(r.outcome.success)!==entry.success||r.outcome.unwanted_contact_pairs!==entry.unwantedContacts)throw Error('Episode result mismatch');
  const validCell=cell=>Array.isArray(cell)&&cell.length===2&&cell.every(Number.isInteger)&&cell[0]>=0&&cell[0]<w&&cell[1]>=0&&cell[1]<h;
  if(!validCell(r.start_xy)||!validCell(r.goal_xy)||!r.static_walls_xy.every(validCell))throw Error('Invalid maze geometry');
  const walls=new Set(r.static_walls_xy.map(c=>c.join(','))),decoded=[];
  let contactTotal=0,paid=0;
  for(let i=0;i<r.steps.length;i++){
   const f=r.steps[i],event=f.event;
   if(f.step!==i||event?.step!==i||!validCell(f.robot_position_after_action_xy)||!Number.isFinite(f.robot_heading_after_action_rad))throw Error('Invalid recorded frame '+i);
   if(i===0?event.event_type!=='initial_state':!['addition','relocation','removal'].includes(event.event_type))throw Error('Unexpected environment event at '+i);
   const objects=f.objects_after,pedestrians=f.pedestrians_xy;
   if(!Array.isArray(objects)||!Array.isArray(pedestrians)||!objects.every(o=>validCell(o.cell)&&typeof o.object_id==='string')||!pedestrians.every(o=>validCell(o.cell)&&typeof o.pedestrian_id==='string'))throw Error('Invalid recorded hazards');
   if((f.paid_observation_count!==paid+Number(f.observation_granted))||f.paid_observation_count>entry.scanLimit)throw Error('Paid scan mismatch');
   paid=f.paid_observation_count;
   contactTotal+=Object.values(f.unwanted_contacts||{}).reduce((s,n)=>s+n,0);
   decoded.push({...f,memory:decodeMemory(f.spatial_memory_ternary_packed_2bit_base64,w,h),
     truth:decodeBits(f.ground_truth_static_and_object_occupancy_packed_bits_base64,w,h)});
  }
  const last=decoded.at(-1);
  if(paid!==entry.paidScans||contactTotal!==entry.unwantedContacts||Number(last.goal_reached)!==entry.success)throw Error('Final frame does not match indexed outcome');
  return {replay:r,frames:decoded,walls};
 }
 function rawURL(index,entry){return 'https://raw.githubusercontent.com/'+index.sourceRepo+'/'+index.sourceCommit+'/'+index.sourcePath+'replays/primary/'+entry.filename}
 async function loadReplay(index,entry,signal){
  if(!globalThis.crypto?.subtle||!globalThis.DecompressionStream)throw Error('Replay needs a modern browser over HTTPS or localhost.');
  const res=await fetch(rawURL(index,entry),{signal});
  if(!res.ok)throw Error('Replay download failed (HTTP '+res.status+')');
  const bytes=await res.arrayBuffer();
  if(bytes.byteLength!==entry.bytes)throw Error('Replay file size mismatch');
  const hash=Array.from(new Uint8Array(await crypto.subtle.digest('SHA-256',bytes)),b=>b.toString(16).padStart(2,'0')).join('');
  if(hash!==entry.sha256)throw Error('Replay SHA-256 verification failed');
  const stream=new Blob([bytes]).stream().pipeThrough(new DecompressionStream('gzip'));
  const body=JSON.parse(await new Response(stream).text());
  return validateReplay(body,entry,index);
 }
 root.H2Replay={decodeMemory,decodeBits,getEntry,validateIndex,validateReplay,rawURL,loadReplay};
})(globalThis);
