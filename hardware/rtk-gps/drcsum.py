import re,sys
t=open('drc.rpt').read()
for blk in t.split('\n[')[1:]:
    k=blk.split(']')[0]
    if k in ('silk_over_copper',): continue
    lines=[l.strip() for l in blk.split('\n')[2:4]]
    print(k[:14], ' | '.join(lines)[:230])
