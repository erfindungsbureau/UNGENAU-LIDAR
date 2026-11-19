#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""flaechen_glaetten.py -- Messrauschen auf ebenen Flaechen entfernen (Waende, Boden, Decke, Tische).

Der Ouster Gen1 misst jeden Punkt nur auf ~3 cm genau; Waende erscheinen dadurch 6-10 cm dick.
Hier wird die Wolke in Wuerfel (ZELLE, Standard 0.1 m) geteilt; pro Wuerfel wird aus ihm und seinen
26 Nachbarn (also ~3*ZELLE Umgebung) die beste Ebene bestimmt. Ist diese Umgebung eindeutig eben
(kleinster Eigenwert < EBEN * mittlerer, Rest-Streuung < MAX_SIGMA), werden die Punkte des Wuerfels
auf die Ebene gezogen -- aber nur die, die naeher als 3 Sigma liegen (ein Bild an der Wand bleibt).
Kanten, Ecken, Moebel: Umgebung nicht eben -> Punkte bleiben, wie sie sind.

Aufruf: python flaechen_glaetten.py EIN.ply AUS.ply [ZELLE=0.1] [EBEN=0.2] [MAX_SIGMA=0.05]
Liest/schreibt binaere PLY (alle Felder bleiben erhalten, nur x y z aendern sich).
stdout: FORTSCHRITT k n (fuer den Balken im Web-UI), am Ende Anteil geglaetteter Punkte.
"""
from __future__ import print_function
import sys
import numpy as np

TYPES = {'float': 'f4', 'float32': 'f4', 'double': 'f8', 'uchar': 'u1', 'uint8': 'u1', 'char': 'i1',
         'short': 'i2', 'ushort': 'u2', 'int': 'i4', 'uint': 'u4', 'int32': 'i4', 'uint32': 'u4'}


def read_ply(path):
    f = open(path, 'rb')
    head = []
    while True:
        line = f.readline()
        head.append(line)
        if line.strip() == b'end_header':
            break
    assert b'binary_little_endian' in b''.join(head), 'nur binaere PLY (little endian)'
    fields = [(l.split()[2].decode(), '<' + TYPES[l.split()[1].decode()]) for l in head
              if l.startswith(b'property ')]
    return head, np.fromfile(f, np.dtype(fields))


def kappe(i):
    return (i[:, 0] << 42) | (i[:, 1] << 21) | i[:, 2]


def glaetten(X, zelle=0.1, eben=0.2, max_sigma=0.05, fortschritt=lambda k, n: None):
    """X (n,3) float64 -> (X geglaettet, Maske der verschobenen Punkte). Aendert X selbst (Speicher, TX2)."""
    o = X.min(0)
    key = np.zeros(len(X), np.int64)
    L = np.empty(X.shape, np.float32)                             # Koordinaten relativ zur Wuerfelmitte
    for k in range(3):                                            # spaltenweise: Spitzenspeicher fuer den TX2
        x = X[:, k] - o[k]
        ck = np.floor(x / zelle).astype(np.int64) + 1             # +1: Nachbar -1 bleibt >= 0
        L[:, k] = x - (ck - 0.5) * zelle
        key |= ck << (42 - 21 * k)
    del x, ck
    keys, inv = np.unique(key, return_inverse=True)
    del key
    m = len(keys)
    kc = np.stack([(keys >> 42) & 0x1FFFFF, (keys >> 21) & 0x1FFFFF, keys & 0x1FFFFF], 1)
    PAARE = [(0, 0), (0, 1), (0, 2), (1, 1), (1, 2), (2, 2)]

    def ebenen(g):
        """Ebene je Wuerfel aus ihm + 26 Nachbarn, Punkte mit Gewicht g -> (Anzahl, Mitte, Eigenwerte, Normale)."""
        S = np.zeros((m, 10))
        S[:, 0] = np.bincount(inv, g, m)
        for a in range(3):                                         # spaltenweise: spart GB auf dem TX2
            S[:, 1 + a] = np.bincount(inv, g * L[:, a], m)
        for j, (a, b) in enumerate(PAARE):
            S[:, 4 + j] = np.bincount(inv, g * L[:, a] * L[:, b], m)
        # Summen der Nachbarn auf die Mitte des Zielwuerfels verschoben: x -> x + d*zelle
        A = np.zeros_like(S)
        for dx in (-1, 0, 1):
            for dy in (-1, 0, 1):
                for dz in (-1, 0, 1):
                    d = np.array([dx, dy, dz])
                    nk = kappe(kc + d)
                    i = np.clip(np.searchsorted(keys, nk), 0, m - 1)
                    ok = keys[i] == nk
                    n, sx, sxx = S[i, 0][ok], S[i, 1:4][ok], S[i, 4:][ok]
                    t = d * zelle
                    A[ok, 0] += n
                    A[ok, 1:4] += sx + n[:, None] * t
                    for j, (a, b) in enumerate(PAARE):            # (x+t)(y+t') = xy + x t' + t y + t t'
                        A[ok, 4 + j] += sxx[:, j] + sx[:, a] * t[b] + sx[:, b] * t[a] + n * t[a] * t[b]
        n = np.maximum(A[:, 0], 1e-9)
        mu = A[:, 1:4] / n[:, None]
        C = np.empty((m, 3, 3))
        for j, (a, b) in enumerate(PAARE):
            C[:, a, b] = C[:, b, a] = A[:, 4 + j] / n - mu[:, a] * mu[:, b]
        w, V = np.linalg.eigh(C)
        return A[:, 0], mu, np.clip(w, 0, None), V[:, :, 0]

    # 1. Durchgang mit allen Punkten, 2. ohne Ausreisser (> 2 Sigma von der ersten Ebene):
    # Fehlmessungen an Kanten ("Faecher") verderben sonst die Ebene der ganzen Umgebung
    def abstand():
        return sum((L[:, k] - mu[inv, k]) * nrm[inv, k] for k in range(3))

    cnt, mu, w, nrm = ebenen(np.ones(len(X)))
    fortschritt(1, 4)
    dist = abstand()
    g = (np.abs(dist) < 2 * np.sqrt(w[inv, 0]) + 0.005).astype(np.float64)
    fortschritt(2, 4)
    cnt, mu, w, nrm = ebenen(g)
    sig = np.sqrt(w[:, 0])
    # eben < 0.3 laesst an Kanten eine schraege Ebene durch Wand + Boden zu (Ecke wird rund); 0.2 nicht
    plan = (cnt >= 30) & (w[:, 0] < eben * w[:, 1]) & (sig < max_sigma)
    fortschritt(3, 4)
    # Punkte auf ihre Ebene ziehen
    pl = plan[inv]
    dist = abstand()
    zieh = pl & (np.abs(dist) < 3 * sig[inv])
    del L, pl
    X[zieh] -= dist[zieh, None] * nrm[inv[zieh]]
    fortschritt(4, 4)
    return X, zieh


def _selbsttest():
    r = np.random.RandomState(0)
    n = 200000
    wand = np.c_[r.uniform(0, 3, n), 0.37 + r.normal(0, 0.03, n), r.uniform(0, 2.5, n)]
    boden = np.c_[r.uniform(0, 3, n), r.uniform(0.37, 3, n), r.normal(0, 0.03, n)]
    X = np.vstack([wand, boden])
    Y, z = glaetten(X.copy())
    w = np.abs(X[:, 2] > 0.5)                                     # Wandpunkte weit weg vom Boden
    y = Y[:n][w[:n], 1]
    mad = 1.4826 * np.median(np.abs(y - np.median(y)))           # robust: Ausreisser > 3 Sigma bleiben roh
    assert mad < 0.004, mad
    assert abs(np.mean(Y[:n][w[:n], 1]) - 0.37) < 0.005
    assert z.mean() > 0.5
    # kein Punkt darf sich von der wahren Oberflaeche (Wand oder Boden) entfernen, auch an der Kante
    def abstand(P):
        return np.minimum(np.abs(P[:, 1] - 0.37), np.abs(P[:, 2]))
    schlechter = (abstand(Y) > abstand(X) + 0.01).mean()
    assert schlechter < 0.001, schlechter


if __name__ == '__main__':
    _selbsttest()
    if len(sys.argv) < 3:
        sys.exit(__doc__)
    ein, aus = sys.argv[1], sys.argv[2]
    zelle = float(sys.argv[3]) if len(sys.argv) > 3 else 0.1
    eben = float(sys.argv[4]) if len(sys.argv) > 4 else 0.2
    max_sigma = float(sys.argv[5]) if len(sys.argv) > 5 else 0.05
    head, P = read_ply(ein)
    X = np.stack([P['x'], P['y'], P['z']], 1)                     # float32 reicht (Mikrometer auf 30 m)

    def fs(k, n):
        print('FORTSCHRITT %d %d' % (k, n)); sys.stdout.flush()
    Y, zieh = glaetten(X, zelle, eben, max_sigma, fs)
    for j, a in enumerate('xyz'):
        P[a] = Y[:, j]
    with open(aus, 'wb') as f:
        for line in head:
            if line.startswith(b'end_header'):
                f.write(b'comment Flaechen geglaettet (flaechen_glaetten.py, Zelle %.2f m)\n' % zelle)
            f.write(line)
        P.tofile(f)
    print('Geglaettet: %.1f %% der Punkte auf Flaechen gezogen -> %s' % (100.0 * zieh.mean(), aus))
