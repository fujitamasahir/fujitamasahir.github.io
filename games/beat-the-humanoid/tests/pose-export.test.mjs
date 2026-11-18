import {readFileSync} from 'node:fs';
const code=readFileSync(new URL('../mujoco-bridge.js',import.meta.url),'utf8');
const snippet=code.slice(code.indexOf(' function poseFor('),code.indexOf(' function renderAssets()'));
const data={geom_xpos:new Float64Array([1,2,3,4,5,6]),geom_xmat:new Float64Array([1,0,0,0,1,0,0,0,1,0,-1,0,1,0,0,0,0,1])};
const run=new Function('data','let drawingIds=[0,1];const check=()=>{},toJson=JSON.stringify;'+snippet+';return JSON.parse(renderPose())');
const result=run(data);
if(result.length!==24||result[0]!==1||result[12]!==4)throw Error('Pose export regression');
console.log('Pose export regression passed');
