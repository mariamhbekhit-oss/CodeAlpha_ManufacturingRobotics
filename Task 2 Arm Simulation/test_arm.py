import unittest,math
from arm_simulation import ik,fk,trajectory,PICK,PLACE,HOME
class ArmTests(unittest.TestCase):
    def test_roundtrip_all_samples(self):
        for r in trajectory(): self.assertLess(math.dist(fk((r['q1'],r['q2'])),(r['target_x'],r['target_y'])),1e-9)
    def test_unreachable(self):
        for target in [(500,0),(0,0)]:
            with self.assertRaises(ValueError): ik(*target)
    def test_grip_and_release(self):
        rows=trajectory()
        for r in rows:
            if r['holding']: self.assertLess(math.dist((r['obj_x'],r['obj_y']),(r['x'],r['y'])),1e-9)
        self.assertEqual((rows[-1]['obj_x'],rows[-1]['obj_y']),PLACE)
        self.assertEqual((rows[-1]['x'],rows[-1]['y']),HOME)
    def test_transfer_continuity(self):
        rows=trajectory()
        self.assertLess(max(math.dist((a['x'],a['y']),(b['x'],b['y'])) for a,b in zip(rows,rows[1:])),8.)
if __name__=='__main__': unittest.main(verbosity=2)
