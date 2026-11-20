
import os,sys,time,numpy as np
base=os.path.expanduser('~/BodySweepProto');sys.path.insert(0,base)
sys.path.insert(0,os.path.expanduser('~/.local/share/FreeCAD/v26-3/Mod/PimpMyFreeCAD'))
cache=base+'/cache'
def log(s): open(cache+'/bench.txt','a').write('%s %s\n'%(time.strftime('%H:%M:%S'),s))
try:
    import pda_bodysweep_raytraced as rbs
    import sdf_close_test as T, sdf_v2_test as V2
    L1=V2.lib_sf(1.0)
    shapes={}
    for name in ('cola','torus','hammer'):
        tool,path,_=L1['shapes'](name);shapes[name]=(tool,path)
    for name in ('cola','torus','hammer'):
        tool,path=shapes[name]
        log('%s tool vol %.0f'%(name,tool.Volume))
        for meth in ('fast','standard','coarse'):
            t0=time.time()
            try:
                sol,info=rbs.build(tool,path,method=meth)
                log('%s %s: %.1f s valid=%s solids=%d vol=%.0f warn=%s'%(name,meth,time.time()-t0,sol.isValid(),len(sol.Solids),sol.Volume,str(info.get('warnings'))[:80]))
            except Exception as e:
                log('%s %s: FAIL %.1f s %s'%(name,meth,time.time()-t0,str(e)[:80]))
    # SDF configurations
    for cfg,(sf,hd) in (('SDF hizli (SF1,hdiv30)',(1.0,30)),('SDF kaliteli (SF2,hdiv60)',(2.0,60))):
        L=V2.lib_sf(sf)
        for name in ('cola','torus','hammer'):
            tool,path,_=L['shapes'](name)
            bb=tool.BoundBox;size=max(bb.XLength,bb.YLength,bb.ZLength);h0=size/hd
            t0=time.time();D,lo,h=L['sweep_sdf'](tool,path,h0,lambda m:None)
            t1=time.time();D2,_=V2.local_closing(D,h);v,t=L['marching_tets'](D2,lo,h);m,_=T.to_mesh_clean(v,t)
            log('%s %s: alan %.1f s, kapama+mesh %.1f s, toplam %.1f s'%(cfg,name,t1-t0,time.time()-t1,time.time()-t0))
    log('DONE')
except Exception as e:
    import traceback;log('ERROR '+traceback.format_exc())
