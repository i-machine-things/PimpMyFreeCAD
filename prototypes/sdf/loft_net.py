
import numpy as np
from scipy.ndimage import gaussian_filter1d

def slice_mesh(V,F,p0,n):
    d=(V-p0)@n;sg=d>0;ts=sg[F];cut=np.where(ts.any(1)&~ts.all(1))[0]
    pts=[];key={};segs=[]
    def ip(a,b):
        k=(min(a,b),max(a,b))
        if k not in key:
            t=d[a]/(d[a]-d[b]);key[k]=len(pts);pts.append(V[a]+t*(V[b]-V[a]))
        return key[k]
    for ti in cut:
        t=F[ti];e=[ip(t[i],t[(i+1)%3]) for i in range(3) if sg[t[i]]!=sg[t[(i+1)%3]]]
        if len(e)==2:segs.append(e)
    adj={}
    for a,b in segs: adj.setdefault(a,[]).append(b);adj.setdefault(b,[]).append(a)
    seen=set();loops=[];P=np.array(pts)
    for s0 in adj:
        if s0 in seen: continue
        lp=[s0];seen.add(s0);prev=None;cur=s0
        while True:
            nx=[x for x in adj[cur] if x!=prev and x not in seen]
            if not nx:break
            prev,cur=cur,nx[0];seen.add(cur);lp.append(cur)
        loops.append(P[lp])
    return loops

def perim(P): return np.linalg.norm(P-np.roll(P,-1,0),axis=1).sum()

def orient(P,n):
    a=np.cross(P,np.roll(P,-1,0)).sum(0)@n
    return P if a>0 else P[::-1]

def resample_closed(P,step):
    Q=np.vstack([P,P[:1]]);seg=np.linalg.norm(np.diff(Q,axis=0),axis=1);s=np.r_[0,np.cumsum(seg)];n=max(16,int(s[-1]/step))
    u=np.linspace(0,s[-1],n,endpoint=False);return np.stack([np.interp(u,s,Q[:,k]) for k in range(3)],1)

def find_corners(P,K=None,win=5,thr=35,nms=6):
    n=len(P);Ps=np.stack([gaussian_filter1d(P[:,k],1.5,mode='wrap') for k in range(3)],1)
    a=Ps-np.roll(Ps,win,0);b=np.roll(Ps,-win,0)-Ps
    cs=(a*b).sum(1)/(np.linalg.norm(a,axis=1)*np.linalg.norm(b,axis=1)+1e-12);ang=np.degrees(np.arccos(np.clip(cs,-1,1)))
    order=np.argsort(-ang);pk=[]
    for i in order:
        if ang[i]<thr: break
        if all(min((i-j)%n,(j-i)%n)>nms for j in pk): pk.append(int(i))
        if K and len(pk)==K: break
    return sorted(pk),ang

def build_rows(loops_per_slice,snapC,step=1.25,Ncorner=None,target=8.0,snaprad=3.5):
    """loops_per_slice: list of oriented closed loops (one per slice). returns grid (NS,N,3), corner column indices"""
    R=[resample_closed(l,step) for l in loops_per_slice]
    ks=[find_corners(r) [0] for r in R]
    cnt=[len(k) for k in ks]
    K=Ncorner or int(np.bincount(cnt).argmax())
    ks=[find_corners(r,K)[0] for r in R]
    # per-slice start corner: track
    starts=[];prev=None
    for r,k in zip(R,ks):
        pts=r[k]
        if len(k)!=K: starts.append(None);continue
        if prev is None: i0=0
        else: i0=int(np.argmin(np.linalg.norm(pts-prev,axis=1)))
        prev=pts[i0];starts.append(i0)
    # segment counts from median slice
    good=[i for i,s in enumerate(starts) if s is not None]
    mid=good[len(good)//2];r=R[mid];k=ks[mid];i0=starts[mid];k=k[i0:]+k[:i0]
    def arcs(r,k):
        n=len(r);out=[]
        for j in range(len(k)):
            a=k[j];b=k[(j+1)%len(k)];idx=[(a+t)%n for t in range(((b-a)%n)+1)];seg=r[idx]
            out.append(seg)
        return out
    L=[np.linalg.norm(np.diff(sg,axis=0),axis=1).sum() for sg in arcs(r,k)]
    cnts=[max(1,int(round(l/target))) for l in L]
    rows=[];ok=[]
    for r,k,i0 in zip(R,ks,starts):
        if i0 is None: ok.append(False);rows.append(None);continue
        k=k[i0:]+k[:i0];sgs=arcs(r,k);row=[]
        for sg,c in zip(sgs,cnts):
            cum=np.r_[0,np.cumsum(np.linalg.norm(np.diff(sg,axis=0),axis=1))];u=np.linspace(0,cum[-1],c,endpoint=False)
            row+= [np.array([np.interp(x,cum,sg[:,q]) for q in range(3)]) for x in u]
        row=np.array(row)
        if snapC is not None:
            colidx=np.r_[0,np.cumsum(cnts)[:-1]]
            from scipy.spatial import cKDTree
            tr=snapC if hasattr(snapC,'query') else cKDTree(snapC)
            d,ii=tr.query(row[colidx])
            for c,dd,i in zip(colidx,d,ii):
                if dd<snaprad: row[c]=tr.data[i]
        rows.append(row);ok.append(True)
    cols=np.r_[0,np.cumsum(cnts)[:-1]]
    return rows,ok,cols,cnts,K

def grid_to_quads(rows,cols):
    NS=len(rows);N=len(rows[0]);V=np.vstack(rows);Q=[];cr=set()
    for i in range(NS-1):
        for j in range(N):
            a=i*N+j;b=i*N+(j+1)%N;c=(i+1)*N+(j+1)%N;d=(i+1)*N+j
            Q.append([a,b,c,d])
    for i in range(NS-1):
        for j in cols:
            a=i*N+j;d=(i+1)*N+j;cr.add((min(a,d),max(a,d)))
    return V,np.array(Q),cr

def cc_creases(V,Q,creases):
    nv=len(V);nf=len(Q);FP=V[Q].mean(1);E={}
    for fi,q in enumerate(Q):
        for i in range(4):
            a,b=q[i],q[(i+1)%4];E.setdefault((min(a,b),max(a,b)),[]).append(fi)
    ek=list(E);eid={e:nv+nf+k for k,e in enumerate(ek)}
    isc={e:((e in creases) or len(E[e])==1) for e in ek}
    EP=np.array([ (V[e[0]]+V[e[1]])/2 if isc[e] else (V[e[0]]+V[e[1]]+FP[E[e][0]]+FP[E[e][1]])/4 for e in ek])
    vcr=[[] for _ in range(nv)]
    for e in ek:
        if isc[e]: vcr[e[0]].append(e[1]);vcr[e[1]].append(e[0])
    Fs=np.zeros_like(V);Rs=np.zeros_like(V);n=np.zeros(nv);ne=np.zeros(nv)
    for fi,q in enumerate(Q):
        for v in q: Fs[v]+=FP[fi];n[v]+=1
    for e in ek:
        m=(V[e[0]]+V[e[1]])/2
        for v in e: Rs[v]+=m;ne[v]+=1
    nV=np.zeros_like(V)
    for v in range(nv):
        c=len(vcr[v])
        if c==2: nV[v]=(6*V[v]+V[vcr[v][0]]+V[vcr[v][1]])/8
        elif c>=3: nV[v]=V[v]
        else: nV[v]=(Fs[v]/n[v]+2*Rs[v]/ne[v]+(n[v]-3)*V[v])/n[v]
    NQ=[]
    for fi,q in enumerate(Q):
        for i in range(4):
            a,b,c=q[i],q[(i+1)%4],q[(i-1)%4]
            NQ.append([a,eid[(min(a,b),max(a,b))],nv+fi,eid[(min(a,c),max(a,c))]])
    ncr=set()
    for e in ek:
        if isc[e] and e in creases:
            m=eid[e];ncr.add((min(e[0],m),max(e[0],m)));ncr.add((min(e[1],m),max(e[1],m)))
    return np.vstack([nV,FP,EP]),np.array(NQ),ncr


def track_corners(R, K, cap=14.0, thr=25):
    """R: list of resampled closed loops. returns list of index arrays (K corners in consistent order) and fallback count."""
    n0=len(R[0]);k0=find_corners(R[0],K)[0]
    tracks=[np.array(k0)];pos=[R[0][k0]];fb=0
    for si in range(1,len(R)):
        r=R[si];cand=find_corners(r,None,thr=thr,nms=5)[0]
        cp=r[cand] if len(cand) else np.zeros((0,3))
        pred=pos[-1] if len(pos)<2 else 2*pos[-1]-pos[-2]
        idx=[];pp=[]
        for j in range(K):
            if len(cp):
                d=np.linalg.norm(cp-pred[j],axis=1);m=int(np.argmin(d))
                if d[m]<cap: idx.append(cand[m]);pp.append(r[cand[m]]);continue
            d=np.linalg.norm(r-pred[j],axis=1);m=int(np.argmin(d));idx.append(m);pp.append(r[m]);fb+=1
        tracks.append(np.array(idx));pos.append(np.array(pp))
    return tracks,fb

def build_rows_tracked(loops_per_slice,snapC=None,step=1.25,K=8,target=8.0):
    R=[resample_closed(l,step) for l in loops_per_slice]
    tracks,fb=track_corners(R,K)
    # validate cyclic order and equal-count arcs from median slice
    def arcs(r,k):
        n=len(r);out=[]
        for j in range(len(k)):
            a=k[j];b=k[(j+1)%len(k)];idx=[(a+t)%n for t in range(((b-a)%n)+1)];out.append(r[idx])
        return out
    bad=[]
    for si,(r,k) in enumerate(zip(R,tracks)):
        n=len(r);d=[(k[(j+1)%K]-k[j])%n for j in range(K)]
        if sum(d)!=n: bad.append(si)   # not cyclically increasing
    mid=len(R)//2;L=[np.linalg.norm(np.diff(sg,axis=0),axis=1).sum() for sg in arcs(R[mid],list(tracks[mid]))]
    cnts=[max(1,int(round(l/target))) for l in L]
    rows=[]
    for r,k in zip(R,tracks):
        sgs=arcs(r,list(k));row=[]
        for sg,c in zip(sgs,cnts):
            cum=np.r_[0,np.cumsum(np.linalg.norm(np.diff(sg,axis=0),axis=1))];u=np.linspace(0,cum[-1],c,endpoint=False)
            row+=[np.array([np.interp(x,cum,sg[:,q]) for q in range(3)]) for x in u]
        rows.append(np.array(row))
    cols=np.r_[0,np.cumsum(cnts)[:-1]]
    return rows,cols,cnts,bad,fb


def cc_poly(V,F,creases):
    """Catmull-Clark on polygon faces F (list of index lists), closed or open; returns V',quads',creases'."""
    nv=len(V);nf=len(F);FP=np.array([V[f].mean(0) for f in F]);E={}
    for fi,f in enumerate(F):
        m=len(f)
        for i in range(m):
            a,b=f[i],f[(i+1)%m];E.setdefault((min(a,b),max(a,b)),[]).append(fi)
    ek=list(E);eid={e:nv+nf+k for k,e in enumerate(ek)}
    isc={e:((e in creases) or len(E[e])==1) for e in ek}
    EP=np.array([(V[e[0]]+V[e[1]])/2 if isc[e] else (V[e[0]]+V[e[1]]+FP[E[e][0]]+FP[E[e][1]])/4 for e in ek])
    vcr=[[] for _ in range(nv)]
    for e in ek:
        if isc[e]: vcr[e[0]].append(e[1]);vcr[e[1]].append(e[0])
    Fs=np.zeros_like(V);Rs=np.zeros_like(V);n=np.zeros(nv);ne=np.zeros(nv)
    for fi,f in enumerate(F):
        for v in f: Fs[v]+=FP[fi];n[v]+=1
    for e in ek:
        m=(V[e[0]]+V[e[1]])/2
        for v in e: Rs[v]+=m;ne[v]+=1
    nV=np.zeros_like(V)
    for v in range(nv):
        c=len(vcr[v])
        if c==2: nV[v]=(6*V[v]+V[vcr[v][0]]+V[vcr[v][1]])/8
        elif c>=3: nV[v]=V[v]
        else: nV[v]=(Fs[v]/n[v]+2*Rs[v]/ne[v]+(n[v]-3)*V[v])/n[v]
    NQ=[]
    for fi,f in enumerate(F):
        m=len(f)
        for i in range(m):
            a,b,c=f[i],f[(i+1)%m],f[(i-1)%m]
            NQ.append([a,eid[(min(a,b),max(a,b))],nv+fi,eid[(min(a,c),max(a,c))]])
    ncr=set()
    for e in ek:
        if isc[e] and e in creases:
            m=eid[e];ncr.add((min(e[0],m),max(e[0],m)));ncr.add((min(e[1],m),max(e[1],m)))
    return np.vstack([nV,FP,EP]),np.array(NQ),ncr

def directed_conflicts(F):
    seen={};bad=0
    for f in F:
        m=len(f)
        for i in range(m):
            e=(f[i],f[(i+1)%m])
            if e in seen: bad+=1
            seen[e]=1
    return bad

def mesh_volume_poly(V,F):
    vol=0.0
    for f in F:
        for i in range(1,len(f)-1):
            a,b,c=V[f[0]],V[f[i]],V[f[i+1]];vol+=np.dot(a,np.cross(b,c))/6
    return vol
