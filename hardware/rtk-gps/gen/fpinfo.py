import pcbnew, sys
sys.path.insert(0,'/w/gen')
from design import PARTS
FP='/usr/share/kicad/footprints/'
seen=set()
for p in PARTS:
    if p['fp'] in seen: continue
    seen.add(p['fp'])
    lib,name=p['fp'].split(':')
    f=pcbnew.FootprintLoad(FP+lib+'.pretty',name)
    bb=f.GetCourtyard(pcbnew.F_CrtYd).BBox() if f.GetCourtyard(pcbnew.F_CrtYd).OutlineCount() else f.GetBoundingBox()
    print(f"{name:60s} {pcbnew.ToMM(bb.GetWidth()):6.2f} x {pcbnew.ToMM(bb.GetHeight()):6.2f}  c=({pcbnew.ToMM(bb.GetCenter().x):.2f},{pcbnew.ToMM(bb.GetCenter().y):.2f})")
    if name in ('ublox_NEO','QFN-60-1EP_7x7mm_P0.4mm_EP3.4x3.4mm','USB_C_Receptacle_GCT_USB4105-xx-A_16P_TopMnt_Horizontal','JST_GH_SM06B-GHS-TB_1x06-1MP_P1.25mm_Horizontal','U.FL_Hirose_U.FL-R-SMT-1_Vertical','LGA-16_3x3mm_P0.5mm'):
        print('   ', [(pd.GetNumber(), round(pcbnew.ToMM(pd.GetPosition().x),2), round(pcbnew.ToMM(pd.GetPosition().y),2)) for pd in f.Pads()][:30])
