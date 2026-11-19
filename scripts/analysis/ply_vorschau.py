#!/usr/bin/env python3
"""ply_vorschau.py -- leichte 3D-Vorschau eines fertigen Modells fuer das Web-UI.

Aus der 1-cm-Wolke (PLY, binaer) werden Wuerfel zu RES (Standard 4 cm), im selben Format wie die
Live-Karte: Int16-Tripel (Wuerfelindex x y z, Mitte = (i + 0.5) * RES). Dazu eine kleine JSON-Datei
mit Aufloesung, Anzahl und Bodenhoehe (fuer die Hoehenfarben).

Aufruf: python3 ply_vorschau.py EIN.ply AUS_PREFIX [RES=0.04]   -> AUS_PREFIX.bin, AUS_PREFIX.json
"""
import json
import os
import sys
import numpy as np

TYPES = {'float': 'f4', 'float32': 'f4', 'double': 'f8', 'uchar': 'u1', 'uint8': 'u1', 'char': 'i1',
         'short': 'i2', 'ushort': 'u2', 'int': 'i4', 'uint': 'u4', 'int32': 'i4', 'uint32': 'u4'}


def ply_punkte(path):
    """x, y, z einer binaeren PLY als Speicherabbild (nichts wird ganz eingelesen)."""
    with open(path, 'rb') as f:
        head = b''
        while not head.endswith(b'end_header\n'):
            line = f.readline()
            if not line:
                raise ValueError('kein PLY-Kopf')
            head += line
    assert b'binary_little_endian' in head, 'nur binaere PLY (little endian)'
    fields = [(l.split()[2].decode(), '<' + TYPES[l.split()[1].decode()]) for l in head.split(b'\n')
              if l.startswith(b'property ')]
    return np.memmap(path, np.dtype(fields), 'r', offset=len(head))


def vorschau(P, res):
    q = [np.floor(P[a] / res).astype(np.int32) for a in 'xyz']
    ok = (np.abs(q[0]) < 32000) & (np.abs(q[1]) < 32000) & (np.abs(q[2]) < 32000)
    key = ((q[0][ok].astype(np.int64) + 32768) << 32) | ((q[1][ok].astype(np.int64) + 32768) << 16) \
        | (q[2][ok].astype(np.int64) + 32768)
    key = np.unique(key)
    W = np.stack([(key >> 32) - 32768, ((key >> 16) & 0xFFFF) - 32768, (key & 0xFFFF) - 32768], 1)
    return W.astype('<i2')


def _selbsttest():
    P = np.zeros(4, [('x', '<f4'), ('y', '<f4'), ('z', '<f4')])
    P['x'] = [0.01, 0.02, -0.05, 1.01]; P['z'] = [0, 0, 0, -1.99]
    W = vorschau(P, 0.04)
    assert len(W) == 3 and [-2, 0, 0] in W.tolist() and [25, 0, -50] in W.tolist(), W.tolist()


if __name__ == '__main__':
    _selbsttest()
    if len(sys.argv) < 3:
        sys.exit(__doc__)
    ein, aus = sys.argv[1], sys.argv[2]
    res = float(sys.argv[3]) if len(sys.argv) > 3 else 0.04
    W = vorschau(ply_punkte(ein), res)
    z0 = float(np.percentile(W[:, 2], 2) + 0.5) * res if len(W) else 0.0      # ungefaehr der Boden
    with open(aus + '.bin.part', 'wb') as f:
        f.write(W.tobytes())
    os.rename(aus + '.bin.part', aus + '.bin')
    with open(aus + '.json', 'w') as f:                                         # zuletzt: Zeichen "fertig"
        json.dump({'res': res, 'n': int(len(W)), 'z0': round(z0, 2), 'quelle': os.path.basename(ein)}, f)
    print('Vorschau: %d Wuerfel a %.0f cm -> %s.bin' % (len(W), res * 100, aus))
