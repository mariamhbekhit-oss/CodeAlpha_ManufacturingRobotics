"""Scan-based conveyor sorting behavioral model. Not a safety-rated PLC program."""
from dataclasses import dataclass
@dataclass
class Inputs:
    start:bool=False
    stop:bool=False
    reset:bool=False
    safe_ok:bool=True
    overload_ok:bool=True
    station:bool=False
    exit:bool=False
    extended:bool=False
    retracted:bool=True
class Controller:
    def __init__(self):
        self.state='IDLE'; self.timer=0.; self.count=0; self.reason=''; self.prev=Inputs()
        self.motor=False; self.extend=False
    def scan(self,i,dt=.02):
        if dt<=0: raise ValueError('Positive scan time required')
        start_edge=i.start and not self.prev.start; reset_edge=i.reset and not self.prev.reset
        exit_edge=i.exit and not self.prev.exit
        previous_state=self.state
        if not i.safe_ok or not i.overload_ok:
            self.state='FAULT'; self.reason='Safety chain or overload'
        elif i.stop:
            # Stop during a part cycle requires clearing the cell before reset.
            if self.state not in ('IDLE','RUN','FAULT') or i.station:
                self.state='FAULT'; self.reason='Interrupted cycle; clear cell'
            elif self.state!='FAULT': self.state='IDLE'
        elif self.state=='FAULT':
            if reset_edge and not i.station and not i.exit and i.retracted and not i.extended:
                self.state='IDLE'; self.reason=''
        elif i.extended and i.retracted:
            self.state='FAULT'; self.reason='Contradictory cylinder sensors'
        elif self.state=='IDLE':
            if start_edge and not i.station and not i.exit and i.retracted and not i.extended: self.state='RUN'
        elif self.state=='RUN':
            if i.station: self.state='SETTLE'
        elif self.state=='SETTLE':
            if self.timer>=.20: self.state='EXTEND'
        elif self.state=='EXTEND':
            if i.extended: self.state='RETRACT'
            elif self.timer>=1.: self.state='FAULT'; self.reason='Extend timeout'
        elif self.state=='RETRACT':
            if i.retracted: self.state='VERIFY'
            elif self.timer>=1.: self.state='FAULT'; self.reason='Retract timeout'
        elif self.state=='VERIFY':
            if exit_edge and not i.station:
                self.count+=1; self.state='RUN'
            elif self.timer>=2.: self.state='FAULT'; self.reason='Exit timeout / part not cleared'
        self.timer=0. if self.state!=previous_state else self.timer+dt
        self.motor=self.state=='RUN'; self.extend=self.state=='EXTEND'
        if not i.safe_ok or not i.overload_ok or i.stop or self.state=='FAULT': self.motor=self.extend=False
        self.prev=Inputs(**vars(i))
        return dict(state=self.state,motor=self.motor,extend=self.extend,count=self.count,reason=self.reason)
