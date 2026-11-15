const CDN='https://cdn.jsdelivr.net/pyodide/v0.27.7/full/';
const files=['__init__','simulation','observation','scheduling','memory','navigation','evaluation'].map(n=>'spatial_memory/'+n+'.py').concat('browser_engine.py');
const ready=(async()=>{
 importScripts(CDN+'pyodide.js');
 const p=await loadPyodide({indexURL:CDN});
 await p.loadPackage('numpy');p.FS.mkdirTree('/game/spatial_memory');
 await Promise.all(files.map(async file=>{const r=await fetch('./python/'+file);if(!r.ok)throw Error('Missing '+file);p.FS.writeFile('/game/'+file,await r.text())}));
 p.runPython("import sys; sys.path.insert(0, '/game'); from browser_engine import browser_episode");return p;
})();
ready.then(()=>postMessage({type:'ready'}),e=>postMessage({type:'error',error:String(e)}));
self.onmessage=async({data})=>{try{const p=await ready,fn=p.globals.get('browser_episode');try{postMessage({id:data.id,value:JSON.parse(fn(JSON.stringify(data.payload)))})}finally{fn.destroy()}}catch(e){postMessage({id:data.id,error:String(e)})}};
