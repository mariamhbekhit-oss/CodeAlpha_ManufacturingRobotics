import unittest
from conveyor_controller import Controller,Inputs
class ConveyorTests(unittest.TestCase):
    def advance(self,c,i,n=65):
        for _ in range(n): c.scan(i)
    def cycle(self,c):
        c.scan(Inputs(start=True)); c.scan(Inputs(station=True)); self.advance(c,Inputs(station=True),15)
        self.assertEqual(c.state,'EXTEND')
        c.scan(Inputs(station=True,extended=True,retracted=False))
        self.assertEqual(c.state,'RETRACT')
        c.scan(Inputs(station=False,retracted=True)); self.assertEqual(c.state,'VERIFY')
        c.scan(Inputs(exit=True)); self.assertEqual(c.state,'RUN')
    def test_nominal_and_no_double_count(self):
        c=Controller(); self.cycle(c); self.assertEqual(c.count,1)
        self.advance(c,Inputs(exit=True),10); self.assertEqual(c.count,1)
    def test_safety_every_active_state(self):
        for state in ['RUN','SETTLE','EXTEND','RETRACT','VERIFY']:
            c=Controller(); c.state=state; c.scan(Inputs(safe_ok=False))
            self.assertEqual(c.state,'FAULT'); self.assertFalse(c.motor or c.extend)
    def test_overload(self):
        c=Controller(); c.scan(Inputs(start=True)); c.scan(Inputs(overload_ok=False)); self.assertEqual(c.state,'FAULT')
    def test_extend_jam(self):
        c=Controller(); c.state='EXTEND'; self.advance(c,Inputs(retracted=False)); self.assertEqual(c.state,'FAULT')
    def test_retract_jam(self):
        c=Controller(); c.state='RETRACT'; self.advance(c,Inputs(retracted=False)); self.assertEqual(c.state,'FAULT')
    def test_exit_jam(self):
        c=Controller(); c.state='VERIFY'; self.advance(c,Inputs(),120); self.assertEqual(c.state,'FAULT')
    def test_held_start_no_restart(self):
        c=Controller(); c.scan(Inputs(start=True)); c.scan(Inputs(start=True,safe_ok=False))
        c.scan(Inputs(start=True,reset=True)); self.assertEqual(c.state,'IDLE')
        c.scan(Inputs(start=True)); self.assertEqual(c.state,'IDLE')
        c.scan(Inputs()); c.scan(Inputs(start=True)); self.assertEqual(c.state,'RUN')
    def test_blocked_reset(self):
        c=Controller(); c.state='FAULT'; c.scan(Inputs(reset=True,station=True)); self.assertEqual(c.state,'FAULT')
    def test_sensor_contradiction(self):
        c=Controller(); c.scan(Inputs(extended=True,retracted=True)); self.assertEqual(c.state,'FAULT')
    def test_interrupted_cycle(self):
        c=Controller(); c.state='EXTEND'; c.scan(Inputs(stop=True)); self.assertEqual(c.state,'FAULT')
if __name__=='__main__': unittest.main(verbosity=2)
