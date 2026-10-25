import pcbnew
b=pcbnew.LoadBoard('/w/rtk-gps/rtk_gps.kicad_pcb')
zs=b.Zones()
print(type(zs), len(zs))
for i in range(len(zs)):
    print(type(zs[i]))
print([type(z) for z in b.Zones()])
