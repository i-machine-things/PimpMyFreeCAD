
import os,sys,time,pickle,numpy as np
base=os.path.expanduser('~/BodySweepProto');sys.path.insert(0,base)
cache=base+'/cache';os.makedirs(cache,exist_ok=True)
def status(s): open(cache+'/status3.txt','a').write('%s %s\n'%(time.strftime('%H:%M:%S'),s))
def read_obj(p):
    V=[];F=[]
    for l in open(p):
        t=l.split()
        if not t: continue
        if t[0]=='v': V.append([float(x) for x in t[1:4]])
        elif t[0]=='f': F.append([int(x.split('/')[0])-1 for x in t[1:]])
    return np.array(V),F
try:
    from scipy import ndimage
    import sdf_close_test as T, sdf_v2_test as V2
    import loft_net as LN
    for name in ('cola','torus'):
        status(name+' start')
        L=V2.lib_sf(2.0);tool,path,_=L['shapes'](name)
        bb=tool.BoundBox;size=max(bb.XLength,bb.YLength,bb.ZLength);h0=size/60
        D,lo,h=L['sweep_sdf'](tool,path,h0,lambda m:None)
        D2,_n=V2.local_closing(D,h)
        np.save(cache+'/D2_%s.npy'%name,D2);np.savez(cache+'/meta_%s.npz'%name,lo=lo,h=h);status(name+' field saved h=%.3f'%h)
        v,t=L['marching_tets'](D2,lo,h);m,_=T.to_mesh_clean(v,t);pts,fac=m.Topology
        Vr=np.array(pts,float);Fr=np.array(fac,int)
        refvol=sum(np.dot(Vr[a],np.cross(Vr[b],Vr[c]))/6 for a,b,c in Fr)
        Vc,Fc=read_obj(base+'/remesh_out/%s_s2/%s_s2_rem_p0_0_quadrangulation_smooth.obj'%(name,name))
        status(name+' cage verts %d faces %d'%(len(Vc),len(Fc)))
        cr=set();Vq=Vc;Fq=[list(f) for f in Fc]
        for k in range(2): Vq,Fq,cr=LN.cc_poly(Vq,Fq,cr);Fq=[list(f) for f in Fq]
        status(name+' CC2 verts %d'%len(Vq))
        def samp(Fld,P): return ndimage.map_coordinates(Fld,((P-lo)/h).T,order=1,mode='nearest')
        def grad(Fld,P):
            g=np.zeros_like(P);e=0.5*h
            for k in range(3):
                dv=np.zeros(3);dv[k]=e;g[:,k]=(samp(Fld,P+dv)-samp(Fld,P-dv))/(2*e)
            return g
        def stats(P,tag):
            d=np.abs(samp(D2,P));vol=LN.mesh_volume_poly(P,Fq)
            nm=np.cross(P[[f[1] for f in Fq]]-P[[f[0] for f in Fq]],P[[f[3] for f in Fq]]-P[[f[0] for f in Fq]]);nm/=np.maximum(np.linalg.norm(nm,axis=1),1e-12)[:,None]
            status('%s %s |D| mean %.3f p95 %.3f max %.3f vol %.2f%% (abs vol %.0f)'%(name,tag,d.mean(),np.percentile(d,95),d.max(),100*(abs(vol)-abs(refvol))/abs(refvol),abs(vol)))
        stats(Vq,'before')
        Fs=ndimage.gaussian_filter(np.clip(D2,-6*h,6*h),1.0)
        bad=np.abs(samp(D2,Vq))>0.6;zone=bad.copy()
        nb=[set() for _ in range(len(Vq))]
        for f in Fq:
            for i in range(len(f)):
                a,b=f[i],f[(i+1)%len(f)];nb[a].add(b);nb[b].add(a)
        for _ in range(4):
            idx=np.where(zone)[0];new=zone.copy()
            for i in idx:
                for j in nb[i]: new[j]=True
            zone=new
        w=zone.astype(float);P=Vq.copy()
        for _ in range(5):
            d=samp(Fs,P);g=grad(Fs,P);n2=(g*g).sum(1)+1e-9;s=(d/n2)[:,None]*g
            mm=np.linalg.norm(s,axis=1);s*=np.minimum(1,h/np.maximum(mm,1e-9))[:,None];P=P-w[:,None]*s
        status('%s zone %d of %d'%(name,zone.sum(),len(Vq)))
        stats(P,'after')
        pickle.dump((Vq,P,Fq,zone),open(cache+'/wrap_%s.pkl'%name,'wb'))
    status('DONE')
except Exception as e:
    import traceback;status('ERROR '+traceback.format_exc())
