"""Original educational 2R planar pick-and-place simulation. Units: mm, radians, seconds."""
import math, json, csv
from pathlib import Path
L1,L2=250.0,180.0
HOME=(300.,180.); PICK=(250.,60.); PLACE=(100.,240.)
DT=0.05

def fk(q):
    a,b=q
    return (L1*math.cos(a)+L2*math.cos(a+b),L1*math.sin(a)+L2*math.sin(a+b))

def ik(x,y):
    c=(x*x+y*y-L1*L1-L2*L2)/(2*L1*L2)
    if abs(c)>1+1e-12: raise ValueError('Target outside workspace')
    b=math.acos(max(-1,min(1,c)))
    a=math.atan2(y,x)-math.atan2(L2*math.sin(b),L1+L2*math.cos(b))
    if not (-math.radians(100)<=a<=math.pi and math.radians(5)<=b<=math.radians(170)):
        raise ValueError('Joint limit violated')
    return a,b

def trajectory():
    phases=[('Approach',HOME,PICK,2.,False),('Grasp',PICK,PICK,.5,True),
            ('Transfer',PICK,PLACE,3.,True),('Release',PLACE,PLACE,.5,False),
            ('Return',PLACE,HOME,2.,False)]
    out=[]; t=0.; obj=PICK
    for name,start,end,duration,holding in phases:
        n=round(duration/DT)
        for k in range(n):
            s=k/n; u=3*s*s-2*s*s*s
            target=tuple(start[j]+u*(end[j]-start[j]) for j in range(2))
            q=ik(*target); tip=fk(q)
            if holding: obj=tip
            elif name in ('Release','Return'): obj=PLACE
            out.append(dict(t=round(t,5),phase=name,q1=q[0],q2=q[1],x=tip[0],y=tip[1],
                            target_x=target[0],target_y=target[1],holding=holding,obj_x=obj[0],obj_y=obj[1]))
            t+=DT
    q=ik(*HOME)
    out.append(dict(t=round(t,5),phase='Complete',q1=q[0],q2=q[1],x=HOME[0],y=HOME[1],
                    target_x=HOME[0],target_y=HOME[1],holding=False,obj_x=PLACE[0],obj_y=PLACE[1]))
    return out

def export(outdir=None):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.animation import FuncAnimation, PillowWriter
    from matplotlib.patches import Circle,Rectangle
    p=Path(outdir or Path(__file__).parent); p.mkdir(exist_ok=True,parents=True)
    rows=trajectory()
    with (p/'trajectory.csv').open('w',newline='') as f:
        w=csv.DictWriter(f,fieldnames=list(rows[0])); w.writeheader(); w.writerows(rows)
    (p/'trajectory.json').write_text(json.dumps(rows))
    fig,ax=plt.subplots(figsize=(8,6)); fig.patch.set_facecolor('#f1f5f9')
    ax.set(xlim=(-90,450),ylim=(-90,450),xlabel='x (mm)',ylabel='y (mm)',aspect='equal')
    ax.grid(alpha=.2); ax.add_patch(Circle((0,0),430,fill=False,linestyle='--',alpha=.3))
    ax.add_patch(Circle((0,0),70,fill=False,linestyle='--',alpha=.3))
    ax.scatter([PICK[0],PLACE[0]],[PICK[1],PLACE[1]],s=170,marker='s',color=['#94a3b8','#94a3b8'])
    ax.text(PICK[0]+10,PICK[1]-20,'Pick fixture'); ax.text(PLACE[0]+12,PLACE[1]+15,'Place fixture')
    ax.plot([r['x'] for r in rows],[r['y'] for r in rows],':',color='#94a3b8',label='Planned tip path')
    line,=ax.plot([],[],color='#2563eb',lw=9,marker='o',markersize=11,label='2R manipulator')
    obj=Rectangle((0,0),16,16,color='#f59e0b'); ax.add_patch(obj)
    label=ax.text(.02,.97,'',transform=ax.transAxes,va='top',fontsize=10)
    ax.legend(loc='lower right'); ax.set_title('CodeAlpha | 2R planar pick-and-place | kinematic model')
    def update(i):
        r=rows[i]; ex=L1*math.cos(r['q1']); ey=L1*math.sin(r['q1'])
        line.set_data([0,ex,r['x']],[0,ey,r['y']]); obj.set_xy((r['obj_x']-8,r['obj_y']-8))
        label.set_text(f"t={r['t']:.2f} s | {r['phase']} | grip={'closed' if r['holding'] else 'open'}\nq1={math.degrees(r['q1']):.1f} deg | q2={math.degrees(r['q2']):.1f} deg")
        return line,obj,label
    for name,index in [('01_home',0),('02_grasp',45),('03_transfer',80),('04_placed',130)]:
        update(index); fig.savefig(p/(name+'.png'),dpi=150,bbox_inches='tight')
    animation=FuncAnimation(fig,update,frames=range(len(rows)),interval=50,blit=False)
    animation.save(p/'pick_and_place.gif',writer=PillowWriter(fps=20)); plt.close(fig)
    errors=[math.hypot(r['x']-r['target_x'],r['y']-r['target_y']) for r in rows]
    summary={'samples':len(rows),'duration_s':rows[-1]['t'],'max_fk_tracking_error_mm':max(errors),
             'final_object_error_mm':math.hypot(rows[-1]['obj_x']-PLACE[0],rows[-1]['obj_y']-PLACE[1]),
             'q1_range_deg':[min(math.degrees(r['q1']) for r in rows),max(math.degrees(r['q1']) for r in rows)],
             'q2_range_deg':[min(math.degrees(r['q2']) for r in rows),max(math.degrees(r['q2']) for r in rows)],
             'model':'Ideal kinematics; no dynamics, contact, torque or collision certification'}
    (p/'simulation_results.json').write_text(json.dumps(summary,indent=2)); print(json.dumps(summary,indent=2))
if __name__=='__main__': export()
