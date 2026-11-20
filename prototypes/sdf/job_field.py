
import os,sys,time,pickle,numpy as np
base=os.path.expanduser('~/BodySweepProto');sys.path.insert(0,base)
cache=base+'/cache';os.makedirs(cache,exist_ok=True)
def status(s): open(cache+'/status.txt','a').write('%s %s\n'%(time.strftime('%H:%M:%S'),s))
try:
    from scipy import ndimage
    from scipy.spatial import cKDTree
    import sdf_close_test as T, sdf_v2_test as V2
    status('start')
    L=V2.lib_sf(2.0);tool,path,_=L['shapes']('hammer')
    bb=tool.BoundBox;size=max(bb.XLength,bb.YLength,bb.ZLength);h0=size/60
    D,lo,h=L['sweep_sdf'](tool,path,h0,lambda m:None)
    D2,_n=V2.local_closing(D,h)
    np.save(cache+'/D2.npy',D2);np.savez(cache+'/meta.npz',lo=lo,h=h);status('field saved')
    # corner trajectories + cap rims
    rbs=L['rbs'];pf=rbs.PathFrames(path,tool.BoundBox.Center,step=min(0.5,size/40),mode='rmf',twist_deg=0.0)
    tl=tool.copy();tl.Placement=pf.placement(0).inverse().multiply(tool.Placement)
    def M(s):
        m=pf.placement(s).Matrix;return np.array([[m.A11,m.A12,m.A13,m.A14],[m.A21,m.A22,m.A23,m.A24],[m.A31,m.A32,m.A33,m.A34],[0,0,0,1]])
    ns=int(np.ceil(pf.L/(h/4)))+1;S=np.linspace(0,pf.L,ns);Ms=[M(s) for s in S]
    verts=np.array([[v.X,v.Y,v.Z] for v in tl.Vertexes])
    traj=[np.array([(m[:3,:3]@p+m[:3,3]) for m in Ms]) for p in verts]
    rims=[]
    for m in (Ms[0],Ms[-1]):
        for e in tl.Edges:
            pts=np.array([[p.x,p.y,p.z] for p in e.discretize(max(2,int(e.Length/(h/4))+1))])
            rims.append((m[:3,:3]@pts.T).T+m[:3,3])
    def samp(F,P): return ndimage.map_coordinates(F,((P-lo)/h).T,order=1,mode='nearest')
    curves=[]
    for c in traj+rims:
        d=samp(D2,c);curves.append((c,d,np.abs(d)<1.2*h))
    pickle.dump(curves,open(cache+'/curves.pkl','wb'));status('curves saved')
    # mesh + ridge correction
    v,t=L['marching_tets'](D2,lo,h);m,_=T.to_mesh_clean(v,t);pts,fac=m.Topology;V0=np.array(pts,float);F0=np.array(fac,int)
    sel=[c[ok] for c,d,ok in curves if ok.mean()>=0.9 and -1.0<d[ok].mean()<1.6 and d[ok].std()<0.4];C=np.vstack(sel)
    def grad(P):
        g=np.zeros_like(P);e=0.5*h
        for k in range(3):
            dv=np.zeros(3);dv[k]=e;g[:,k]=(samp(D2,P+dv)-samp(D2,P-dv))/(2*e)
        return g
    dC=samp(D2,C);gC=grad(C);delta=(dC/((gC*gC).sum(1)+1e-9))[:,None]*gC
    n=np.linalg.norm(delta,axis=1);delta*=np.minimum(1,1.2*h/np.maximum(n,1e-9))[:,None]
    tree=cKDTree(C);dd,ii=tree.query(V0,k=6,distance_upper_bound=2.5*1.5*h)
    fin=np.isfinite(dd);w=np.exp(-(np.where(fin,dd,1e9)/(1.5*h))**2);ii=np.where(fin,ii,0)
    dl=(delta[ii]*w[:,:,None]).sum(1)/np.maximum(w.sum(1),1e-12)[:,None];V1=V0+dl*w[:,0][:,None]
    np.savez(cache+'/mesh.npz',V0=V0,V1=V1,F=F0,C=C);status('mesh saved facets %d'%len(F0))
    status('DONE')
except Exception as e:
    import traceback;status('ERROR '+traceback.format_exc())
