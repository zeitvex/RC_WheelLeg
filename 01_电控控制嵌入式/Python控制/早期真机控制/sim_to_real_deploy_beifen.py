#!/usr/bin/env python3
import argparse, sys, threading, time
from dataclasses import dataclass
from pathlib import Path
import numpy as np

sys.path.append('/home/rc2/work/rcwork/control')
from drivers.motor_driver import RobStrideDriver

ROOT = Path('/home/rc2/work/rcwork/wheelleg_deploy_swj/wheelleg_deploy/wheelleg_mjlab/beifen')
sys.path += [str(ROOT), str(ROOT / 'mujoco_sim')]
from sim2real_control_api import DeployState, Sim2RealControlAPI  # type: ignore

sys.path.append('/home/rc2/work/rcwork')
from trajectory_interpolator import TrajectoryInterpolator

LEGS = ('fl','fr','rl','rr')
LJ = ('hip_abduction_joint','hip_pitch_joint','knee_joint')
WJ = 'wheel_joint'
JNS = [f'{l}_{j}' for l in LEGS for j in (*LJ, WJ)]

@dataclass
class Cfg:
    mid:int; model:str; sign:float; off:float; bus:str

class Deploy:
    def __init__(self, can1, can2, hz=100.0, use_interpolation=True, interp_method='quintic', interp_time=1.5):
        self.d1, self.d2 = RobStrideDriver(can1, False), RobStrideDriver(can2, False)
        self.api = Sim2RealControlAPI(); self.dt = 1.0/hz; self.lk = threading.Lock()
        self.run = False; self.enabled = False; self.estop = True
        self.mode='stand'; self.vx=0.0; self.vy=0.0; self.yaw=0.0; self.h=0.33; self.roll=0.0; self.pitch=0.0
        self.prone = False
        self.kp_leg, self.kd_leg, self.kd_wheel = 80.0, 2.5, 2.0
        self.cfg = self._cfg(); self.q=np.zeros(23); self.dq=np.zeros(22)
        
        self.use_interpolation = use_interpolation
        if self.use_interpolation:
            self.interpolator = TrajectoryInterpolator(method=interp_method, transition_time=interp_time)
            print(f'[Interpolation] Enabled: method={interp_method}, transition_time={interp_time}s')
        else:
            self.interpolator = None
            print('[Interpolation] Disabled')

    def _cfg(self):
        sign={'fl_hip_abduction_joint':-1,'fl_hip_pitch_joint':-1,'fl_knee_joint':-1,'fl_wheel_joint':-1,'fr_hip_abduction_joint':-1,'fr_hip_pitch_joint':1,'fr_knee_joint':1,'fr_wheel_joint':1,'rl_hip_abduction_joint':1,'rl_hip_pitch_joint':-1,'rl_knee_joint':-1,'rl_wheel_joint':-1,'rr_hip_abduction_joint':1,'rr_hip_pitch_joint':1,'rr_knee_joint':1,'rr_wheel_joint':1}
        off={'fl_hip_abduction_joint':0.003,'fl_hip_pitch_joint':0.030,'fl_knee_joint':0.028,'fl_wheel_joint':0.0,'fr_hip_abduction_joint':0.004,'fr_hip_pitch_joint':0.038,'fr_knee_joint':0.011,'fr_wheel_joint':0.0,'rl_hip_abduction_joint':0.019,'rl_hip_pitch_joint':-0.034,'rl_knee_joint':0.025,'rl_wheel_joint':0.0,'rr_hip_abduction_joint':-0.001,'rr_hip_pitch_joint':0.039,'rr_knee_joint':0.018,'rr_wheel_joint':0.0}
        ids={'fl_hip_abduction_joint':1,'fl_hip_pitch_joint':2,'fl_knee_joint':3,'fl_wheel_joint':4,'fr_hip_abduction_joint':5,'fr_hip_pitch_joint':6,'fr_knee_joint':7,'fr_wheel_joint':8,'rl_hip_abduction_joint':1,'rl_hip_pitch_joint':2,'rl_knee_joint':3,'rl_wheel_joint':4,'rr_hip_abduction_joint':5,'rr_hip_pitch_joint':6,'rr_knee_joint':7,'rr_wheel_joint':8}
        bus={k:('can1' if k.startswith('f') else 'can2') for k in JNS}
        return {jn:Cfg(ids[jn],'rs-06',float(sign[jn]),float(off[jn]),bus[jn]) for jn in JNS}

    def _drv(self, jn): return self.d1 if self.cfg[jn].bus=='can1' else self.d2

    def connect(self):
        self.d1.connect(); self.d2.connect()
        for jn,c in self.cfg.items(): self._drv(jn).add_motor(jn,c.mid,c.model)

    def enable_all(self):
        for jn in JNS: self._drv(jn).enable(jn)
        with self.lk: self.enabled=True; self.estop=False; self.vx=self.vy=self.yaw=0.0

    def disable_all(self):
        for jn in JNS: self._drv(jn).disable(jn)
        with self.lk: self.enabled=False

    def clear(self):
        for jn in JNS: self._drv(jn).clear_warnings(jn)

    def set_estop(self,on):
        with self.lk:
            self.estop=on
            if on: self.vx=self.vy=self.yaw=0.0
        if on: self.disable_all()

    def set_prone(self,on):
        with self.lk: self.prone=on; self.api.prone=on

    def _update_pin(self,leg,wheel):
        q=np.zeros(23); dq=np.zeros(22); q[2]=self.h; q[6]=1.0
        for i,_ in enumerate(LEGS):
            b=7+i*4; q[b:b+3]=leg[i*3:i*3+3]; dq[6+i*4+3]=wheel[i]
        self.q,self.dq=q,dq

    def step(self):
        with self.lk:
            if self.estop or (not self.enabled): return
            m=self.mode; vx=float(np.clip(self.vx,-0.8,0.8)); vy=float(np.clip(self.vy,-0.5,0.5)); yaw=float(np.clip(self.yaw,-3,3)); h=float(np.clip(self.h,0.157,0.448)); r=float(np.clip(self.roll,-0.4,0.4)); p=float(np.clip(self.pitch,-0.4,0.4)); prone=self.prone
        cm='trot' if m=='trot' else 'wheel'
        if m=='stand': vx=vy=yaw=0.0
        self.api.prone=prone; self.api.set_mode(cm); self.api.set_command(vx,vy,yaw,height=h)
        st=DeployState(rpy=np.array([r,p,0.0]))
        leg,wheel = self.api.compute(st,self.dt,self.q,self.dq) if cm=='trot' else self.api.compute(st,self.dt)
        self._update_pin(leg,wheel); cmd=self.api.to_joint_dict(leg,wheel)
        
        if self.use_interpolation and self.interpolator is not None:
            leg_cmd = {jn: cmd[jn] for jn in JNS if not jn.endswith(WJ)}
            self.interpolator.set_target(leg_cmd)
            smooth_cmd = self.interpolator.update(self.dt)
            for jn in leg_cmd:
                cmd[jn] = smooth_cmd[jn]
        
        for jn in JNS:
            d=self._drv(jn); c=self.cfg[jn]
            if jn.endswith(WJ): d.control_mit(jn,0.0,c.sign*float(cmd.get(jn,0.0)),0.0,self.kd_wheel,0.0)
            else: d.control_mit(jn,c.sign*float(cmd[jn])+c.off,0.0,self.kp_leg,self.kd_leg,0.0)

    def loop(self):
        while self.run:
            t=time.time()
            try: self.step()
            except Exception as e: print('[control]',e)
            time.sleep(max(0.0,self.dt-(time.time()-t)))

    def start(self): self.run=True; threading.Thread(target=self.loop,daemon=True).start()
    def stop(self):
        self.run=False; time.sleep(0.05)
        try: self.disable_all()
        finally: self.d1.disconnect(); self.d2.disconnect()

    def status(self):
        with self.lk: return f'mode={self.mode} en={self.enabled} estop={self.estop} prone={self.prone} vx={self.vx:.2f} vy={self.vy:.2f} yaw={self.yaw:.2f} h={self.h:.3f}'

class CLI:
    def __init__(self,d): self.d=d
    def run(self):
        print('enable disable clear estop_on estop_off prone_on prone_off status')
        print('mode stand|wheel|trot, vx vy yaw h roll pitch, stop, quit')
        print('interp_on interp_off interp_time <sec>, interp_method linear|cubic|quintic|cosine')
        while True:
            try: s=input('cmd> ').strip().lower()
            except (EOFError,KeyboardInterrupt): s='quit'
            if s in ('quit','exit'): break
            if s=='enable': self.d.enable_all(); continue
            if s=='disable': self.d.disable_all(); continue
            if s=='clear': self.d.clear(); continue
            if s=='estop_on': self.d.set_estop(True); continue
            if s=='estop_off': self.d.set_estop(False); continue
            if s=='prone_on': self.d.set_prone(True); continue
            if s=='prone_off': self.d.set_prone(False); continue
            if s=='status': print(self.d.status()); continue
            if s=='stop':
                with self.d.lk: self.d.vx=self.d.vy=self.d.yaw=0.0
                continue
            if s=='interp_on':
                with self.d.lk: self.d.use_interpolation=True
                print('Interpolation enabled'); continue
            if s=='interp_off':
                with self.d.lk: self.d.use_interpolation=False
                print('Interpolation disabled'); continue
            if s.startswith('interp_time '):
                try:
                    t=float(s.split()[1])
                    if self.d.interpolator: self.d.interpolator.set_transition_time(t)
                    print(f'Interpolation time set to {t}s')
                except Exception as e: print(f'Error: {e}')
                continue
            if s.startswith('interp_method '):
                try:
                    method=s.split()[1]
                    if self.d.interpolator: self.d.interpolator.set_method(method)
                    print(f'Interpolation method set to {method}')
                except Exception as e: print(f'Error: {e}')
                continue
            if s.startswith('mode '):
                m=s.split()[1]
                if m in ('stand','wheel','trot'):
                    with self.d.lk: self.d.mode=m
                else: print('bad mode')
                continue
            try:
                k,v=s.split()[0],float(s.split()[1])
                with self.d.lk:
                    if k=='vx': self.d.vx=v
                    elif k=='vy': self.d.vy=v
                    elif k=='yaw': self.d.yaw=v
                    elif k=='h': self.d.h=v
                    elif k=='roll': self.d.roll=v
                    elif k=='pitch': self.d.pitch=v
                    else: print('unknown')
            except Exception: print('unknown/bad')

class GUI:
    def __init__(self,d):
        import tkinter as tk
        from tkinter import ttk
        self.d=d; self.root=tk.Tk(); self.root.title('WheelLeg Deploy')
        f=ttk.Frame(self.root,padding=8); f.grid(row=0,column=0,sticky='nsew')
        self.state=tk.StringVar(value='E-STOP ON'); ttk.Label(f,textvariable=self.state).grid(row=0,column=0,columnspan=4,sticky='w')
        ttk.Button(f,text='Enable',command=self.en).grid(row=1,column=0)
        ttk.Button(f,text='Disable',command=self.dis).grid(row=1,column=1)
        ttk.Button(f,text='E-STOP ON',command=lambda:self.es(True)).grid(row=1,column=2)
        ttk.Button(f,text='E-STOP OFF',command=lambda:self.es(False)).grid(row=1,column=3)
        ttk.Button(f,text='Prone ON',command=lambda:self.pr(True)).grid(row=2,column=2)
        ttk.Button(f,text='Prone OFF',command=lambda:self.pr(False)).grid(row=2,column=3)
        self.mode=tk.StringVar(value='stand'); self.vx=tk.DoubleVar(value=0.0); self.vy=tk.DoubleVar(value=0.0); self.yaw=tk.DoubleVar(value=0.0); self.h=tk.DoubleVar(value=0.33)
        self.roll=tk.DoubleVar(value=0.0); self.pitch=tk.DoubleVar(value=0.0)
        cb=ttk.Combobox(f,textvariable=self.mode,values=['stand','wheel','trot'],state='readonly'); cb.grid(row=3,column=0,columnspan=2,sticky='ew'); cb.bind('<<ComboboxSelected>>',lambda _:self.sync())
        ttk.Button(f,text='Stop',command=self.stp).grid(row=3,column=3)
        self.sl(f,4,'vx',self.vx,-0.8,0.8); self.sl(f,5,'vy',self.vy,-0.5,0.5); self.sl(f,6,'yaw',self.yaw,-3,3); self.sl(f,7,'height',self.h,0.157,0.448); self.sl(f,8,'roll',self.roll,-0.4,0.4); self.sl(f,9,'pitch',self.pitch,-0.4,0.4)
        self.info=tk.StringVar(value=''); ttk.Label(f,textvariable=self.info).grid(row=10,column=0,columnspan=4,sticky='w'); self.tick()
    def sl(self,f,r,n,v,lo,hi):
        from tkinter import ttk
        ttk.Label(f,text=n).grid(row=r,column=0,sticky='w'); ttk.Scale(f,from_=lo,to=hi,variable=v,command=lambda _:self.sync()).grid(row=r,column=1,columnspan=3,sticky='ew')
    def sync(self):
        with self.d.lk:
            self.d.mode=self.mode.get(); self.d.vx=float(self.vx.get()); self.d.vy=float(self.vy.get()); self.d.yaw=float(self.yaw.get()); self.d.h=float(self.h.get()); self.d.roll=float(self.roll.get()); self.d.pitch=float(self.pitch.get())
    def en(self): self.d.enable_all(); self.state.set('Enabled')
    def dis(self): self.d.disable_all(); self.state.set('Disabled')
    def es(self,on): self.d.set_estop(on); self.state.set('E-STOP ON' if on else 'E-STOP OFF')
    def pr(self,on): self.d.set_prone(on)
    def stp(self):
        with self.d.lk: self.d.vx=self.d.vy=self.d.yaw=0.0
        self.vx.set(0.0); self.vy.set(0.0); self.yaw.set(0.0)
    def tick(self): self.info.set(self.d.status()); self.root.after(150,self.tick)
    def run(self): self.root.mainloop()

def main():
    ap=argparse.ArgumentParser()
    ap.add_argument('--port-can1',default='/dev/can1')
    ap.add_argument('--port-can2',default='/dev/can2')
    ap.add_argument('--hz',type=float,default=100.0)
    ap.add_argument('--no-gui',action='store_true')
    ap.add_argument('--no-interp',action='store_true',help='Disable trajectory interpolation')
    ap.add_argument('--interp-method',default='quintic',choices=['linear','cubic','quintic','cosine'],help='Interpolation method')
    ap.add_argument('--interp-time',type=float,default=0.3,help='Interpolation transition time (seconds)')
    a=ap.parse_args()
    d=Deploy(a.port_can1,a.port_can2,a.hz,use_interpolation=not a.no_interp,interp_method=a.interp_method,interp_time=a.interp_time); d.connect(); d.start()
    try:
        cli=CLI(d); t=threading.Thread(target=cli.run,daemon=True); t.start()
        if a.no_gui:
            while t.is_alive(): time.sleep(0.2)
        else: GUI(d).run()
    finally: d.stop()

if __name__=='__main__': main()
