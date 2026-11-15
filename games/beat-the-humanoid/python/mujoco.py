"""Subset of the native Python MuJoCo API, backed by the official JS WASM module.
Only intended for the frozen kinematic G1 research adapters; not a general binding.
"""
import json
from types import SimpleNamespace
import numpy as np
from js import wasmBridge as _bridge

_values=json.loads(_bridge.constants())
mjtObj=SimpleNamespace(**_values['obj'])
mjtGeom=SimpleNamespace(**_values['geom'])

class MjModel:
    @classmethod
    def from_xml_string(cls, xml):
        model=cls()
        info=json.loads(_bridge.create(xml))
        for name,value in info.items():
            setattr(model,name,np.asarray(value) if isinstance(value,list) else value)
        model.geom_size=model.geom_size.reshape(-1,3)
        model.opt=SimpleNamespace(timestep=0.02)  # Actual timestep is set in MJCF.
        return model

class MjData:
    def __init__(self, model):
        self._fields={}
        self._contacts=None
        self.time=0.0

    def _read(self,name,cols):
        if name not in self._fields:
            self._fields[name]=np.asarray(json.loads(_bridge.field(name)),dtype=float).reshape(-1,cols)
        return self._fields[name]

    @property
    def geom_xpos(self): return self._read('geom_xpos',3)
    @property
    def geom_xmat(self): return self._read('geom_xmat',9)
    @property
    def site_xpos(self): return self._read('site_xpos',3)
    @property
    def site_xmat(self): return self._read('site_xmat',9)
    @property
    def contact(self):
        if self._contacts is None:
            self._contacts=[SimpleNamespace(geom1=a,geom2=b) for a,b in json.loads(_bridge.contacts())]
        return self._contacts
    @property
    def ncon(self): return len(self.contact)

def mj_name2id(model,kind,name):
    return int(_bridge.name2id(int(kind),name))

def mj_id2name(model,kind,index):
    return str(_bridge.id2name(int(kind),int(index)))

def mj_resetDataKeyframe(model,data,key):
    payload=json.loads(_bridge.reset(int(key)))
    data.qpos=np.asarray(payload['qpos'],dtype=float)
    data.qvel=np.asarray(payload['qvel'],dtype=float)
    data.mocap_pos=np.asarray(payload['mocap_pos'],dtype=float).reshape(-1,3)
    data._fields.clear()
    data._contacts=None

def mj_forward(model,data):
    _bridge.forward(json.dumps(dict(qpos=data.qpos.tolist(),qvel=data.qvel.tolist(),mocap_pos=data.mocap_pos.reshape(-1).tolist())))
    data._fields.clear()
    data._contacts=None

def _rays(origin,directions,groups,flg_static,bodyexclude,multi,cutoff):
    return json.loads(_bridge.rays(json.dumps(dict(origin=np.asarray(origin).tolist(),
        directions=np.asarray(directions).reshape(-1).tolist(),
        groups=None if groups is None else np.asarray(groups).tolist(),
        static=bool(flg_static),exclude=int(bodyexclude),multi=multi,cutoff=float(cutoff)))))

def mj_ray(model,data,origin,direction,groups,flg_static,bodyexclude,geomid,normal=None):
    result=_rays(origin,direction,groups,flg_static,bodyexclude,False,0)
    geomid[:]=result['ids']
    return float(result['dist'][0])

def mj_multiRay(model,data,origin,directions,groups,flg_static,bodyexclude,geomid,distances,normal,nray,cutoff):
    result=_rays(origin,directions,groups,flg_static,bodyexclude,True,cutoff)
    geomid[:]=result['ids']
    distances[:]=result['dist']
