# -*- coding: utf-8 -*-
"""imu_check.py BAG -- vergleicht Drehraten SBG (/imu/data) und Ouster (/ouster/imu):
Zeitversatz der Zeitstempel (Kreuzkorrelation) und Drehung SBG->Ouster (Kabsch), w_ouster = R * w_sbg."""
import sys
import numpy as np
import rosbag

def load(bag, topic):
    a = [(m.header.stamp.to_sec(), m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z)
         for _, m, _ in bag.read_messages(topics=[topic])]
    return np.array(a)

def kabsch(A, B):
    """R mit B ~ R A (Zeilenvektoren)."""
    H = A.T.dot(B)
    U, S, Vt = np.linalg.svd(H)
    d = np.sign(np.linalg.det(Vt.T.dot(U.T)))
    return Vt.T.dot(np.diag([1, 1, d])).dot(U.T)

_r = np.random.RandomState(1); _A = _r.randn(500, 3)
_R = np.array([[0, -1, 0], [1, 0, 0], [0, 0, 1.0]])
assert np.allclose(kabsch(_A, _A.dot(_R.T)), _R, atol=1e-9)

bag = rosbag.Bag(sys.argv[1])
s, o = load(bag, "/imu/data"), load(bag, "/ouster/imu")
bag.close()
t0 = max(s[0, 0], o[0, 0]); t1 = min(s[-1, 0], o[-1, 0])
grid = np.arange(t0 + 1, t1 - 1, 0.002)
def res(a, g): return np.stack([np.interp(g, a[:, 0], a[:, i]) for i in (1, 2, 3)], axis=1)
O = res(o, grid)
mo = np.linalg.norm(O, axis=1)
best = None
for lag in np.arange(-0.2, 0.2001, 0.002):
    c = np.corrcoef(np.linalg.norm(res(s, grid + lag), axis=1), mo)[0, 1]
    if best is None or c > best[1]:
        best = (lag, c)
lag = best[0]
S = res(s, grid + lag)
sel = mo > 0.3
R = kabsch(S[sel], O[sel])
resid = np.linalg.norm(O[sel] - S[sel].dot(R.T), axis=1)
ang = np.degrees(np.arccos(np.clip((np.trace(R) - 1) / 2, -1, 1)))
print("Proben SBG %d (%.0f Hz), Ouster %d (%.0f Hz)" % (len(s), len(s) / (s[-1, 0] - s[0, 0]), len(o), len(o) / (o[-1, 0] - o[0, 0])))
print("Zeitversatz: SBG-Stempel %.0f ms %s Ouster (lag=%+.3f s, Korrelation %.3f)" %
      (abs(lag) * 1000, "spaeter als" if lag < 0 else "frueher als", lag, best[1]))
print("Drehung SBG->Ouster: %.1f Grad gesamt" % ang)
np.set_printoptions(precision=4, suppress=True)
print(R)
print("Rest nach Ausrichtung: Median %.3f rad/s, 95%% %.3f rad/s (bei |w|>0.3)" % (np.median(resid), np.percentile(resid, 95)))
for name, a in (("SBG", s), ("Ouster", o)):
    print("max |w| %s: %.0f Grad/s" % (name, np.degrees(np.linalg.norm(a[:, 1:], axis=1).max())))
