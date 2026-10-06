"""Dong hoc UR3 (thuan + nghich so, thuan tuy Python) de tranh KDL nhay nhanh khuyu/co tay.

FK da kiem chung voi robot that trong Gazebo: voi q=[-0.29,-1.19,1.44,-1.83,4.71,-0.27] cho tool0=(0.399,-0.002,0.244)
(log 0.40,0.00,0.24) va q=[-0.28,0.34,-1.57,-0.34,4.71,-0.28] cho (0.402,0.001,0.190) (log 0.40,0.00,0.19);
huong tool0 trong base_link trung voi khung DH 6 (khong can xoay bu).
IK la Newton giam chan (DLS) khoi dong tu mot seed, nen luon cho nghiem GAN seed (khong nhay nhanh).
"""
import math

D = [0.1519, 0.0, 0.0, 0.11235, 0.08535, 0.0819]
A = [0.0, -0.24365, -0.21325, 0.0, 0.0, 0.0]
AL = [math.pi / 2, 0.0, 0.0, math.pi / 2, -math.pi / 2, 0.0]

# Tu the "khuyu len" (elbow>0) dung de pick/place: nghiem tai (0.40, 0.00, 0.24) trong log that.
PICK_SEED = [-0.29, -1.19, 1.44, -1.83, -1.57, -0.27]


def _mm(X, Y):
    return [[sum(X[i][k] * Y[k][j] for k in range(4)) for j in range(4)] for i in range(4)]


def _t(th, d, a, al):
    ct, st, ca, sa = math.cos(th), math.sin(th), math.cos(al), math.sin(al)
    return [[ct, -st * ca, st * sa, a * ct], [st, ct * ca, -ct * sa, a * st], [0, sa, ca, d], [0, 0, 0, 1]]


def fk(q):
    """Tra (R 3x3, p) cua tool0 trong base_link."""
    M = [[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]]
    for i in range(6):
        M = _mm(M, _t(q[i], D[i], A[i], AL[i]))
    R = [[-M[0][0], -M[0][1], -M[0][2]], [-M[1][0], -M[1][1], -M[1][2]], [M[2][0], M[2][1], M[2][2]]]
    return R, [-M[0][3], -M[1][3], M[2][3]]


def quat_to_R(qx, qy, qz, qw):
    return [[1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - qz * qw), 2 * (qx * qz + qy * qw)],
            [2 * (qx * qy + qz * qw), 1 - 2 * (qx * qx + qz * qz), 2 * (qy * qz - qx * qw)],
            [2 * (qx * qz - qy * qw), 2 * (qy * qz + qx * qw), 1 - 2 * (qx * qx + qy * qy)]]


def _err(q, p_t, R_t):
    R, p = fk(q)
    e_p = [p_t[i] - p[i] for i in range(3)]
    # sai so huong: 0.5 * sum cot_i(R) x cot_i(R_t)
    e_r = [0.0, 0.0, 0.0]
    for c in range(3):
        a = [R[r][c] for r in range(3)]
        b = [R_t[r][c] for r in range(3)]
        cr = [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]]
        for i in range(3):
            e_r[i] += 0.5 * cr[i]
    return e_p + e_r


def _solve(Mat, v):
    n = len(v)
    M = [row[:] + [v[i]] for i, row in enumerate(Mat)]
    for i in range(n):
        piv = max(range(i, n), key=lambda r: abs(M[r][i]))
        M[i], M[piv] = M[piv], M[i]
        if abs(M[i][i]) < 1e-12:
            return None
        for r in range(i + 1, n):
            f = M[r][i] / M[i][i]
            for c in range(i, n + 1):
                M[r][c] -= f * M[i][c]
    x = [0.0] * n
    for i in range(n - 1, -1, -1):
        x[i] = (M[i][n] - sum(M[i][j] * x[j] for j in range(i + 1, n))) / M[i][i]
    return x


def ik(p_t, R_t, seed, iters=200, lam=0.02):
    """Newton giam chan tu seed. Tra (q, ok). q gan seed; ok neu sai so vi tri<0.2mm va huong<1e-3."""
    q = list(seed)
    for _ in range(iters):
        e = _err(q, p_t, R_t)
        if max(abs(v) for v in e[:3]) < 2e-4 and max(abs(v) for v in e[3:]) < 1e-3:
            return q, True
        h = 1e-6
        J = [[0.0] * 6 for _ in range(6)]
        for j in range(6):
            qq = list(q)
            qq[j] += h
            e2 = _err(qq, p_t, R_t)
            for i in range(6):
                J[i][j] = -(e2[i] - e[i]) / h  # d(e)/dq = -(d pose)/dq -> J pose = -d e/dq
        JT = [[J[i][j] for i in range(6)] for j in range(6)]
        H = [[sum(JT[a][k] * J[k][b] for k in range(6)) + (lam * lam if a == b else 0.0) for b in range(6)]
             for a in range(6)]
        g = [sum(JT[a][k] * e[k] for k in range(6)) for a in range(6)]
        dq = _solve(H, g)
        if dq is None:
            return q, False
        step = max(abs(v) for v in dq)
        if step > 0.3:
            dq = [v * 0.3 / step for v in dq]
        q = [q[i] + dq[i] for i in range(6)]
    e = _err(q, p_t, R_t)
    return q, (max(abs(v) for v in e[:3]) < 2e-4 and max(abs(v) for v in e[3:]) < 1e-3)
