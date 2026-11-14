# -*- coding: utf-8 -*-
"""lc_optimize.py IN.npz OUT_PREFIX [RADIUS MIN_FIT MAX_RMSE MAX_DPOS MAX_DANG] -- (Python 3) Schleifenschluss offline fuer FAST-LIO2.
Stuetzpunkte (0.5 m / 10 Grad) mit lokaler Karte (+-1 s), Schleifen per Punkt-Ebene-ICP, Posengraph (least_squares),
Korrektur je Umdrehung interpoliert. Bewertung: Wanddicke vorher/nachher, Abweichung Start-Ende."""
import sys, time
import numpy as np
from scipy.spatial import cKDTree
from scipy.optimize import least_squares
from scipy.sparse import lil_matrix

def qmat(q):
    x, y, z, w = q
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                     [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])

def exp_so3(v):
    a = np.linalg.norm(v)
    if a < 1e-12:
        return np.eye(3)
    k = v / a
    K = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    return np.eye(3) + np.sin(a) * K + (1 - np.cos(a)) * K.dot(K)

def log_so3(R):
    c = np.clip((np.trace(R) - 1) / 2, -1, 1); a = np.arccos(c)
    w = np.array([R[2, 1]-R[1, 2], R[0, 2]-R[2, 0], R[1, 0]-R[0, 1]])
    if a < 1e-6:
        return 0.5 * w
    return a / (2 * np.sin(a)) * w

_v = np.array([0.3, -0.2, 0.1]); assert np.allclose(log_so3(exp_so3(_v)), _v)

def voxel(p, v):
    k = np.floor(p / v).astype(np.int64)
    key = (k[:, 0] + 100000) * 40000000000 + (k[:, 1] + 100000) * 200000 + (k[:, 2] + 100000)
    uk, inv, n = np.unique(key, return_inverse=True, return_counts=True)
    return np.stack([np.bincount(inv, p[:, i]) / n for i in range(3)], axis=1)

def normals(p, tree, k=12):
    _, idx = tree.query(p, k=k)
    nb = p[idx] - p[idx].mean(axis=1, keepdims=True)
    cov = np.einsum("nki,nkj->nij", nb, nb)
    w, v = np.linalg.eigh(cov)
    return v[:, :, 0], w[:, 0] / np.maximum(w.sum(axis=1), 1e-12)

def icp(src, tgt, tree, nrm, R, t):
    for it, md in enumerate([0.5] * 8 + [0.25] * 8 + [0.1] * 10):
        p = src.dot(R.T) + t
        d, idx = tree.query(p, k=1, distance_upper_bound=md)
        m = np.isfinite(d)
        if m.sum() < 200:
            return None
        pm, q, n = p[m], tgt[idx[m]], nrm[idx[m]]
        r = np.sum((pm - q) * n, axis=1)
        J = np.hstack([np.cross(pm, n), n])
        dx = -np.linalg.solve(J.T.dot(J) + 1e-6 * np.eye(6), J.T.dot(r))
        dR = exp_so3(dx[:3]); R = dR.dot(R); t = dR.dot(t) + dx[3:]
    p = src.dot(R.T) + t
    d, _ = tree.query(p, k=1, distance_upper_bound=0.05)
    m = np.isfinite(d)
    return R, t, m.mean(), np.sqrt(np.mean(d[m] ** 2)) if m.any() else 1.0

def plane_stats(p, cell=0.5):
    c = np.floor(p / cell).astype(np.int64)
    key = (c[:, 0] + 1000) * 4000000 + (c[:, 1] + 1000) * 2000 + (c[:, 2] + 1000)
    o = np.argsort(key); key, p = key[o], p[o]
    uk, st, n = np.unique(key, return_index=True, return_counts=True)
    out = []
    for s, m in zip(st, n):
        if m < 100:
            continue
        ev = np.linalg.eigvalsh(np.cov(p[s:s+m].T))
        if ev[0] < 0.06 * ev[1] and ev[1] > 0.005:
            out.append(np.sqrt(ev[0]))
    return np.array(out) * 1000

# --- Selbsttest ICP: bekannte Verschiebung einer Ecke aus drei Ebenen wiederfinden ---
_r = np.random.RandomState(0)
_g = np.vstack([np.c_[_r.uniform(0, 2, 3000), _r.uniform(0, 2, 3000), np.zeros(3000)],
                np.c_[np.zeros(3000), _r.uniform(0, 2, 3000), _r.uniform(0, 2, 3000)],
                np.c_[_r.uniform(0, 2, 3000), np.zeros(3000), _r.uniform(0, 2, 3000)]])
_tr = cKDTree(_g); _n, _ = normals(_g, _tr)
_Rt, _tt = exp_so3(np.array([0.02, -0.03, 0.04])), np.array([0.05, -0.04, 0.03])
_res = icp((_g - _tt).dot(_Rt), _g, _tr, _n, np.eye(3), np.zeros(3))
assert np.allclose(_res[0], _Rt, atol=1e-3) and np.allclose(_res[1], _tt, atol=1e-3), _res[:2]

# Schleifen-Suche: Radius um die Odometrie-Position; Annahme eines ICP-Ergebnisses
# (Testlauf aufrecht: 2.0 / 0.5 / 0.03 / 0.6 / 8; stark verrutschte Scans: 4.0 / 0.45 / 0.035 / 1.5 / 15)
RADIUS, MIN_FIT, MAX_RMSE, MAX_DPOS, MAX_DANG = [float(v) for v in (sys.argv[3:8] or [2.0, 0.5, 0.03, 0.6, 8])]

T0 = time.time()
D = np.load(sys.argv[1]); out = sys.argv[2]
times, poses, cnt = D["times"], D["poses"], D["cnt"]
pts = D["pts"].astype(np.float64)
ofs = np.concatenate([[0], np.cumsum(cnt)])
N = len(times)
Rs = [qmat(p[3:]) for p in poses]; ts = [np.array(p[:3]) for p in poses]
scan = lambda i: pts[ofs[i]:ofs[i+1]]

# 1) Stuetzpunkte
kf = [0]
for i in range(1, N):
    j = kf[-1]
    if np.linalg.norm(ts[i] - ts[j]) > 0.5 or np.linalg.norm(log_so3(Rs[j].T.dot(Rs[i]))) > np.radians(10):
        kf.append(i)
K = len(kf)
def submap(i):
    acc = []
    for s in range(max(0, i - 10), min(N, i + 11)):
        Rr = Rs[i].T.dot(Rs[s]); tr = Rs[i].T.dot(ts[s] - ts[i])
        acc.append(scan(s)[::2].dot(Rr.T) + tr)
    return voxel(np.concatenate(acc), 0.05)
print("%d Umdrehungen, %d Stuetzpunkte" % (N, K))

# 2) Schleifen
P = np.array([ts[i] for i in kf]); Tm = np.array([times[i] for i in kf])
cache = {}
def get(a):
    if a not in cache:
        sm = submap(kf[a]); tr = cKDTree(sm); n, curv = normals(sm, tr)
        cache[a] = (sm, tr, n)
    return cache[a]
loops = []
tree_kf = cKDTree(P)
for a in range(K):
    cand = [b for b in tree_kf.query_ball_point(P[a], RADIUS) if Tm[a] - Tm[b] > 15.0]
    if not cand:
        continue
    b = min(cand, key=lambda b: np.linalg.norm(P[a] - P[b]))
    ia, ib = kf[a], kf[b]
    R0 = Rs[ib].T.dot(Rs[ia]); t0 = Rs[ib].T.dot(ts[ia] - ts[ib])      # Pose a im Rahmen b (Odometrie)
    src = get(a)[0]; src = src[np.random.RandomState(a).permutation(len(src))[:8000]]
    tgt, tr, nrm = get(b)
    res = icp(src, tgt, tr, nrm, R0, t0)
    if res is None:
        continue
    Rr, tr_, fit, rmse = res
    dang = np.degrees(np.linalg.norm(log_so3(R0.T.dot(Rr)))); dpos = np.linalg.norm(tr_ - t0)
    ok = fit > MIN_FIT and rmse < MAX_RMSE and dpos < MAX_DPOS and dang < MAX_DANG
    print("Paar %3d-%3d  Zeitabstand %5.1f s  Abstand Odometrie %.2f m | ICP: Passung %.2f, Rest %4.1f mm, Korrektur %.3f m %.2f Grad  %s" %
          (b, a, Tm[a]-Tm[b], np.linalg.norm(P[a]-P[b]), fit, rmse*1000, dpos, dang, "ok" if ok else "verworfen"))
    if ok:
        loops.append((b, a, Rr, tr_))
print("Schleifen: %d akzeptiert" % len(loops))

# 3) Posengraph: Unbekannte = Stuetzpunkte 1..K-1 (Stuetzpunkt 0 fest)
edges = []
for a in range(K - 1):
    i, j = kf[a], kf[a+1]
    edges.append((a, a+1, Rs[i].T.dot(Rs[j]), Rs[i].T.dot(ts[j] - ts[i]), 0.01, np.radians(0.3)))
for b, a, Rr, tr_ in loops:
    edges.append((b, a, Rr, tr_, 0.02, np.radians(0.5)))
x0 = np.zeros(6 * (K - 1))
for a in range(1, K):
    x0[6*(a-1):6*(a-1)+3] = log_so3(Rs[kf[a]]); x0[6*(a-1)+3:6*a] = ts[kf[a]]
def pose(x, a):
    if a == 0:
        return Rs[kf[0]], ts[kf[0]]
    v = x[6*(a-1):6*a]; return exp_so3(v[:3]), v[3:]
def resid(x):
    r = np.empty(6 * len(edges))
    for e, (i, j, Zr, Zt, st, sr) in enumerate(edges):
        Ri, ti = pose(x, i); Rj, tj = pose(x, j)
        Rrel = Ri.T.dot(Rj); trel = Ri.T.dot(tj - ti)
        r[6*e:6*e+3] = log_so3(Zr.T.dot(Rrel)) / sr
        r[6*e+3:6*e+6] = Zr.T.dot(trel - Zt) / st
    return r
S = lil_matrix((6 * len(edges), 6 * (K - 1)), dtype=int)
for e, (i, j, *_) in enumerate(edges):
    for a in (i, j):
        if a > 0:
            S[6*e:6*e+6, 6*(a-1):6*a] = 1
sol = least_squares(resid, x0, jac_sparsity=S, loss="soft_l1", f_scale=3.0, x_scale="jac", max_nfev=60)
print("Optimierung: Kosten %.1f -> %.1f (%s)" % (0.5 * np.sum(resid(x0) ** 2), sol.cost, sol.message[:40]))

# 4) Korrektur je Umdrehung (zwischen Stuetzpunkten interpoliert) und Bewertung
Cr = []; Ct = []
for a in range(K):
    Rn, tn = pose(sol.x, a); Ro, to = Rs[kf[a]], ts[kf[a]]
    C = Rn.dot(Ro.T); Cr.append(log_so3(C)); Ct.append(tn - C.dot(to))
Cr, Ct, kt = np.array(Cr), np.array(Ct), np.array([times[i] for i in kf])
before, after, export = [], [], []
for s in range(N):
    w = scan(s).dot(Rs[s].T) + ts[s]
    cr = np.array([np.interp(times[s], kt, Cr[:, k]) for k in range(3)])
    ct = np.array([np.interp(times[s], kt, Ct[:, k]) for k in range(3)])
    wc = w.dot(exp_so3(cr).T) + ct
    r = np.linalg.norm(scan(s), axis=1)
    sel = r < 8.0
    before.append(w[sel][::2]); after.append(wc[sel][::2]); export.append(wc[r < 10.0])
for name, arr in (("vorher (FAST-LIO2)", before), ("nachher (Schleifenschluss)", after)):
    d = plane_stats(np.concatenate(arr))
    print("%-27s: %4d ebene Wuerfel, Wanddicke Median %.1f mm, 90%% %.1f mm, dick (>30 mm) %.1f %%" %
          (name, len(d), np.median(d), np.percentile(d, 90), 100.0 * (d > 30).mean()))
Rl, tl = pose(sol.x, K - 1); R0_, t0_ = pose(sol.x, 0)
print("Abweichung Start-Ende: vorher %.3f m / %.1f Grad, nachher %.3f m / %.1f Grad" %
      (np.linalg.norm(ts[kf[-1]] - ts[kf[0]]), np.degrees(np.linalg.norm(log_so3(Rs[kf[0]].T.dot(Rs[kf[-1]])))),
       np.linalg.norm(tl - t0_), np.degrees(np.linalg.norm(log_so3(R0_.T.dot(Rl))))))
np.savetxt(out + "_lc.xyz", voxel(np.concatenate(export), 0.03), fmt="%.3f")
np.savetxt(out + "_lc.xyz.path", np.array([pose(sol.x, a)[1] for a in range(K)]), fmt="%.3f")
print("Rechenzeit %.0f s" % (time.time() - T0))
