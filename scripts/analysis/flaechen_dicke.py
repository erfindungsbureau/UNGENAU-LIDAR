#!/usr/bin/env python3
"""flaechen_dicke.py -- wie dick sind ebene Flaechen in einer Punktwolke? (Vergleich vorher/nachher)

Teilt die Wolke in Wuerfel (Standard 0.5 m). Pro Wuerfel Hauptachsenanalyse: ist er eben
(kleinster Eigenwert < 1/10 des mittleren, mind. 200 Punkte), gilt sqrt(kleinster Eigenwert) als
Dicke (Standardabweichung quer zur Flaeche). Getrennt nach Wand (Normale waagrecht) und
Boden/Decke (Normale senkrecht). Doppelte Waende durch Drift machen Wuerfel dicker.

Aufruf: python3 flaechen_dicke.py PREFIX [PREFIX2 ...] [--zelle 0.5]   (liest PREFIX_xyz.f32)
"""
import sys
import numpy as np

args = [a for a in sys.argv[1:] if not a.startswith('--')]
ZELLE = float(sys.argv[sys.argv.index('--zelle') + 1]) if '--zelle' in sys.argv else 0.5
args = [a for a in args if a != str(ZELLE)]


def messen(prefix):
    X = np.fromfile(prefix + '_xyz.f32', np.float32).reshape(-1, 3).astype(np.float64)
    X -= X.mean(0)
    k = np.floor(X / ZELLE).astype(np.int64)
    k -= k.min(0)
    m = k.max(0) + 1
    _, inv, cnt = np.unique((k[:, 0] * m[1] + k[:, 1]) * m[2] + k[:, 2], return_inverse=True, return_counts=True)
    S = np.zeros((len(cnt), 9))
    for j, v in enumerate([X[:, 0], X[:, 1], X[:, 2], X[:, 0] * X[:, 0], X[:, 0] * X[:, 1], X[:, 0] * X[:, 2],
                           X[:, 1] * X[:, 1], X[:, 1] * X[:, 2], X[:, 2] * X[:, 2]]):
        S[:, j] = np.bincount(inv, v, len(cnt))
    del X, k, inv
    ok = cnt >= 200
    S, n = S[ok], cnt[ok].astype(float)[:, None]
    mu = S[:, :3] / n
    C = np.empty((len(n), 3, 3))
    idx = [(0, 0, 3), (0, 1, 4), (0, 2, 5), (1, 1, 6), (1, 2, 7), (2, 2, 8)]
    for a, b, j in idx:
        C[:, a, b] = C[:, b, a] = S[:, j] / n[:, 0] - mu[:, a] * mu[:, b]
    w, V = np.linalg.eigh(C)                       # aufsteigend
    eben = w[:, 0] < 0.1 * w[:, 1]
    dicke = np.sqrt(np.clip(w[:, 0], 0, None))
    nz = np.abs(V[:, 2, 0])                        # z-Anteil der Normalen
    wand, boden = eben & (nz < 0.3), eben & (nz > 0.9)
    return dicke, wand, boden


if __name__ == '__main__' and not args:
    sys.exit(__doc__)
for p in args:
    d, wand, boden = messen(p)
    print(p)
    for name, s in (('Waende', wand), ('Boden/Decke', boden)):
        x = d[s] * 1000
        print('  %-12s %5d Wuerfel  Dicke (1 Sigma) median %5.1f mm   75%% %5.1f mm   90%% %5.1f mm'
              % (name, s.sum(), np.median(x), np.percentile(x, 75), np.percentile(x, 90)))
