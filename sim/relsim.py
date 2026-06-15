import numpy as np
from collections import deque
from dataclasses import replace
from rider_params import DEFAULT
from rider_env import RiderBalanceEnv

d=np.load('ppo_v_pure_export.npz')
mean,var,eps,clip=d['mean'],d['var'],float(d['eps']),float(d['clip'])
W0,b0,W1,b1,W2,b2=d['W0'],d['b0'],d['W1'],d['b1'],d['W2'],d['b2']
def infer(o):
    z=np.clip((o-mean)/np.sqrt(var+eps),-clip,clip)
    x=np.tanh(W0@z+b0); x=np.tanh(W1@x+b1); return float(np.clip((W2@x+b2)[0],-1,1))
DEG=0.017453292

def run(direction, Vcmd=0.30, T=12.0, t_rel=3.0, latency=0.040,
        K=5.0, KI=2.5, KD=10.0, VLP=0.8, AMAX=0.8):
    env=RiderBalanceEnv(params=replace(DEFAULT,latency_s=latency),add_noise=False,frame_stack=1)
    env.reset(seed=1); dt=env.ctrl_dt
    p,pr,x,xv,wv=env._raw_state(); x0=x
    tgt=x; tgtE=x; vdes=0.0; xvelF=0.0; prevX=x; pint=0.0; pintE=0.0
    stack=deque(maxlen=2); log=[]
    n=int(T/dt)
    for i in range(n):
        p,pr,x,xv,wv=env._raw_state()
        xvel=(x-prevX)/dt; prevX=x; xvelF=VLP*xvelF+(1-VLP)*xvel
        driving = (i*dt < t_rel)
        Vc = direction*Vcmd if driving else 0.0
        damax=AMAX*dt
        if abs(Vc)>0.001:
            if Vc>vdes+damax: vdes+=damax
            elif Vc<vdes-damax: vdes-=damax
            else: vdes=Vc
            tgt=x; tgtE=x; pintE=0.0           # home tracks current; no windup
            wasdrive=True
        else:
            if 'wasdrive' in dir() and wasdrive: vdes=0.0; wasdrive=False
            # hold: profile to latched tgt (dist=0) -> just brake; integral when near
            posErr0=x-tgtE
            if abs(posErr0)<0.05: pintE=np.clip(pintE+posErr0*dt,-2,2)
        posErr=x-tgtE
        bias=np.clip(K*posErr + KI*pintE + KD*(xvelF - vdes), -5.0, 5.0)
        prp=p+bias*DEG; pint=np.clip(pint+prp*dt,-1,1)
        frame=np.array([prp,pr,0,xv,wv,pint,0],np.float32)
        if i==0:
            for _ in range(2): stack.append(frame)
        else: stack.append(frame)
        a=infer(np.concatenate(stack)); env.step(np.array([a],np.float32))
        log.append((i*dt, x-x0, tgtE-x0, vdes, p))
    return np.array(log), x0

for dirn,tag in ((+1,'FWD'),(-1,'REV')):
    L,_=run(dirn)
    rel=int(3.0/ (12.0/len(L)))
    home=L[-1,2]
    post=L[L[:,0]>3.0]
    over=max((w[1]-home) if dirn>0 else (home-w[1]) for w in post)
    settle=L[-1,1]
    print('%s: home=%.3f  overshoot-past-home=%+.0fcm  settle=%+.0fcm-from-home'%(tag,home,over*100,(settle-home)*100))
