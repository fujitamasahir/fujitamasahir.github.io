"""Run from the game directory with the native Python environment (NumPy).
This tests the grid session state machine without requiring the browser or MuJoCo.
"""
import json
import sys
import unittest
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'python'))
import browser_engine as eng

class LiveSessions(unittest.TestCase):
    def tearDown(self):
        eng.session_close()

    def start(self, objects=None):
        maze=dict(id='test',width=7,height=7,start=[1,1],goal=[5,5],
                  wall_rows=['1111111','1001001','1001001','1000001','1001001','1001001','1111111'])
        q=dict(maze=maze,objects=objects or [],pedestrians=[],difficulty='easy',
               strategy='periodic',backend='grid2d-proxy',seed=42001,
               rules=dict(step_limit=80,max_static_objects=4,max_pedestrians=2,max_observations=20))
        eng.session_start(json.dumps(q))
        return json.loads(eng.session_next())['frames'][0]

    def test_edit_rejects_only_passage_and_preserves_initial_memory(self):
        f=self.start()
        for objects,people in [([[3,3]],[]),([],[[3,3]])]:
            request=dict(step=0,objects=objects,pedestrians=people)
            self.assertFalse(json.loads(eng.session_edit(json.dumps(request)))['valid'])
            self.assertFalse(json.loads(eng.session_edit(json.dumps(request),True))['valid'])
        self.assertEqual(eng._session['frame'],f)
        request=dict(step=0,objects=[[4,4]],pedestrians=[])
        self.assertTrue(json.loads(eng.session_edit(json.dumps(request),True))['valid'])
        self.assertEqual(eng._session['frame'],f)
        nxt=json.loads(eng.session_next())['frames'][0]
        self.assertEqual(nxt['step'],1)
        self.assertEqual(nxt['scene_revision'],1)
        self.assertEqual(nxt['objects'],[[4,4]])
        self.assertGreaterEqual(nxt['observations'],f['observations'])

    def test_unchanged_resume_keeps_revision(self):
        f=self.start()
        request=dict(step=0,objects=f['objects'],pedestrians=f['people'])
        self.assertTrue(json.loads(eng.session_edit(json.dumps(request),True))['valid'])
        nxt=json.loads(eng.session_next())['frames'][0]
        self.assertEqual(nxt['scene_revision'],0)
        self.assertEqual(nxt['step'],1)

    def test_skip_batches_retain_every_step(self):
        first=self.start()
        steps=[first['step']]
        while True:
            data=json.loads(eng.session_next(16))
            steps.extend(f['step'] for f in data['frames'])
            if 'result' in data:
                self.assertEqual(steps,list(range(data['result']['elapsed_steps']+1)))
                self.assertLessEqual(data['result']['observation_count'],20)
                break

if __name__=='__main__':
    unittest.main()
