# Tabulates the transient buffers of one recording from the LUMA_MEMSTATS lines of a tools/memprof/run.sh log: bytes per cell by stage before and after Luma's sharing.
import re,sys,collections
def parse(path, rec=0):
    T=[];C={};S=[];P=[];SUM=None
    for l in open(path,errors='replace'):
        m=re.match(r'^(T|S|P) (\d+) id=(\d+) class=(\d+) logical=(\d+) first=(\d+) last=(\d+) writer=(.*?)(?: owner=(\d+))?(?: usage=.*)?$',l.rstrip())
        if m and int(m.group(2))==rec:
            k=m.group(1); d=dict(id=int(m.group(3)),cls=int(m.group(4)),log=int(m.group(5)),first=int(m.group(6)),last=int(m.group(7)),writer=m.group(8),owner=m.group(9))
            {'T':T,'S':S,'P':P}[k].append(d); continue
        m=re.match(r'^C (\d+) (\d+) (.*) size=(\[.*?\]) wg=',l)
        if m and int(m.group(1))==rec: C[int(m.group(2))]=(m.group(3),m.group(4))
        if l.startswith('SUM %d '%rec): SUM=l.strip()
    return T,C,S,P,SUM
def short(lbl):
    lbl=re.sub(r'ParamView \{.*?\}\s*\}\s*\}','PV',lbl)
    m=re.match(r'^(Fused(?:Solve)?(?:\[\d+\])?|Reduce|[A-Za-z_]+)',lbl)
    return m.group(1) if m else lbl[:20]
def ranges_for(C):
    sol=[(i,eval(C[i][1])[0]) for i in C if short(C[i][0])=='FusedSolve']
    sizes=set(s for _,s in sol)
    main=[i for i,s in sol if any(abs(2*s-o)<=2 for o in sizes) or True]
    # main sweeps: solves before the SST ones: SST ones are the last 3 FusedSolve
    sst=[i for i,_ in sol[-3:]]
    main=[i for i,_ in sol[:-3]]
    end=main[-1]
    sh=end-68
    r=[(0,8,'1 inviscid: prims, ghosts, face states, fluxes'),(9,25,'2 viscous: wall state, gradients, face fluxes, lambda, lhs prims'),(26,31,'3 block assembly FL/FR (3 dirs x 50 face arrays)'),(32,56,'4 DDADI diagonal (25 cell arrays)'),(57,end,'5 sweeps: line solves, defect r, du'),(end+1,end+9,'6 small wall-line arrays'),(end+10,end+19,'7 SST: gradients, blends, face data, cell terms'),(end+20,end+22,'8 SST line solves'),(end+23,end+33,'9 SST update, forces, norms'),(end+34,end+60,'10 state update')]
    return r
def table(path,rec,cells):
    T,C,S,P,SUM=parse(path,rec)
    ranges=ranges_for(C)
    def stage(i):
        for a,b,name in ranges:
            if a<=i<=b: return name
        return '?'
    pre=collections.defaultdict(lambda:[0,0,0]); post=collections.defaultdict(lambda:[0,0])
    own=collections.defaultdict(list)
    for t in T: own[int(t['owner'])].append(t)
    def key(t):
        w=t['writer']; i=int(w.split(':')[0]) if w!='-' else t['first']
        name=stage(i)
        if short(w.split(':',1)[1] if w!='-' else '')=='FusedSolve' and t['cls']>=8388608: name=name.split(' ')[0]+' line-solve scratch (Thomas upper blocks, 25 f32/cell)'
        return name
    for t in T:
        k=key(t); pre[k][0]+=1; pre[k][1]+=t['cls']; pre[k][2]+=t['log']
    for o,ms in own.items():
        t0=[t for t in ms if t['id']==o][0]; k=key(t0); post[k][0]+=1; post[k][1]+=t0['cls']
    return T,C,S,P,SUM,pre,post
def show(path,rec,cells):
    T,C,S,P,SUM,pre,post=table(path,rec,cells)
    print(path,rec,len(C),'cmds',SUM)
    tot=[0,0,0]
    for k,v in sorted(pre.items()):
        print(f"{k:75s} n={v[0]:4d} pre class {v[1]/1e6:7.1f} MB {v[1]/cells:6.0f} B/c | logical {v[2]/1e6:7.1f} MB {v[2]/cells:6.0f} B/c | kept {post[k][0]:3d} bufs {post[k][1]/1e6:7.1f} MB {post[k][1]/cells:6.0f} B/c")
        for j in range(3): tot[j]+=v[j]
    print(f"{'TOTAL':75s} n={sum(v[0] for v in pre.values()):4d} pre class {tot[1]/1e6:7.1f} MB {tot[1]/cells:6.0f} B/c | logical {tot[2]/1e6:7.1f} MB {tot[2]/cells:6.0f} B/c | kept {sum(v[0] for v in post.values()):3d} {sum(v[1] for v in post.values())/1e6:7.1f} MB {sum(v[1] for v in post.values())/cells:6.0f} B/c")
if __name__ == '__main__':
    import sys
    if len(sys.argv) != 4:
        sys.exit('usage: analyze.py LOG REC CELLS   (LOG: the output of tools/memprof/run.sh; REC: recording number of the "REC n" lines; CELLS: cells of the grid)')
    show(sys.argv[1], int(sys.argv[2]), int(sys.argv[3]))
