import sys, collections
from pymavlink import DFReader
fn=sys.argv[1]; t_lo=float(sys.argv[2]); t_hi=float(sys.argv[3]); step=float(sys.argv[4]) if len(sys.argv)>4 else 0.5
f=DFReader.DFReader_binary(fn)
rows=collections.defaultdict(dict)
while True:
    m=f.recv_msg()
    if m is None: break
    t=m.get_type()
    if t not in ("RCOU","CTUN","ATT","BAT","PSCN","PSCE","OF","VIBE","RFND","XKF4","BARO","RATE","MOTB"): continue
    ts=m.TimeUS/1e6
    if ts<t_lo or ts>t_hi: continue
    k=round(ts/step)*step
    d=rows[k]
    if t=="RCOU": d.update(c1=m.C1,c2=m.C2,c3=m.C3,c4=m.C4)
    elif t=="CTUN": d.update(Alt=m.Alt,BAlt=m.BAlt,SAlt=m.SAlt,ThO=m.ThO,ThH=m.ThH)
    elif t=="ATT": d.update(R=m.Roll,P=m.Pitch,dR=m.DesRoll,dP=m.DesPitch)
    elif t=="BAT": d.update(V=m.Volt,I=m.Curr)
    elif t=="PSCN": d.update(PN=m.PN,TPN=m.TPN,VN=m.VN,TVN=m.TVN)
    elif t=="PSCE": d.update(PE=m.PE,TPE=m.TPE,VE=m.VE,TVE=m.TVE)
    elif t=="OF": d.update(fx=m.flowX,fy=m.flowY,bx=m.bodyX,by=m.bodyY,q=m.Qual)
    elif t=="RFND": d.update(rf=m.Dist)
    elif t=="VIBE": d.update(vx=m.VibeX,vy=m.VibeY,vz=m.VibeZ,clip=m.Clip)
    elif t=="XKF4": d.update(ss=m.SS)
print(" t     c1   c2   c3   c4   ThO  Alt   SAlt  BAlt  rf   R     P     dR    dP   VN    VE    TVN   TVE   fx    fy    bx    by   q  V     I")
for k in sorted(rows):
    d=rows[k]; g=lambda n,fmt: format(d.get(n,float('nan')),fmt)
    print(f"{k:6.1f} {g('c1','4.0f')} {g('c2','4.0f')} {g('c3','4.0f')} {g('c4','4.0f')} {g('ThO','.2f')} {g('Alt','5.2f')} {g('SAlt','5.2f')} {g('BAlt','5.2f')} {g('rf','4.2f')} {g('R','5.1f')} {g('P','5.1f')} {g('dR','5.1f')} {g('dP','5.1f')} {g('VN','5.2f')} {g('VE','5.2f')} {g('TVN','5.2f')} {g('TVE','5.2f')} {g('fx','5.2f')} {g('fy','5.2f')} {g('bx','5.2f')} {g('by','5.2f')} {g('q','3.0f')} {g('V','5.2f')} {g('I','4.0f')}")
