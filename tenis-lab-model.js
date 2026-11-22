/**
 * Tênis Lab - Núcleo Biomecânico, Cinemático e Físico (Desacoplado do DOM)
 * Extraído e modularizado a partir da versão Me23.
 */
(function (global) {
  'use strict';

  const rad = (a) => (a * Math.PI) / 180;
  const deg = (r) => (r * 180) / Math.PI;
  const clampBio = (v, mn, mx) => Math.max(mn, Math.min(mx, v));
  const sm3 = (u) => u * u * (3 - 2 * u);
  const sstep = (a, b, x) => {
    const u = Math.min(1, Math.max(0, (x - a) / (b - a || 1)));
    return u * u * (3 - 2 * u);
  };

  // ===== Álgebra Vetorial 3D =====
  const V = {
    add: (a, b) => [a[0] + b[0], a[1] + b[1], a[2] + b[2]],
    sub: (a, b) => [a[0] - b[0], a[1] - b[1], a[2] - b[2]],
    mul: (a, k) => [a[0] * k, a[1] * k, a[2] * k],
    dot: (a, b) => a[0] * b[0] + a[1] * b[1] + a[2] * b[2],
    len: (a) => Math.hypot(a[0], a[1], a[2]),
    norm(a) {
      const l = Math.hypot(a[0], a[1], a[2]) || 1;
      return [a[0] / l, a[1] / l, a[2] / l];
    },
    cross: (a, b) => [
      a[1] * b[2] - a[2] * b[1],
      a[2] * b[0] - a[0] * b[2],
      a[0] * b[1] - a[1] * b[0],
    ],
  };

  function rotAroundAxis(v, k, theta) {
    const c = Math.cos(theta), s = Math.sin(theta);
    const kxv = V.cross(k, v);
    const kdv = V.dot(k, v);
    return V.add(V.add(V.mul(v, c), V.mul(kxv, s)), V.mul(k, kdv * (1 - c)));
  }

  function clampVecCone(dir, refDir, maxAngRad) {
    const d = V.norm(dir), r = V.norm(refDir);
    const c = V.dot(d, r), minC = Math.cos(maxAngRad);
    if (c >= minC) return d;
    const ortho = V.norm(V.sub(d, V.mul(r, c)));
    return V.norm(V.add(V.mul(r, minC), V.mul(ortho, Math.sin(maxAngRad))));
  }

  // ===== Cinemática Inversa (IK) Analítica 2 Segmentos =====
  function ik(a, t, l1, l2, pole) {
    const d = V.sub(t, a);
    const L = Math.max(0.05, Math.min(V.len(d), l1 + l2 - 0.002));
    const n = V.norm(d);
    const x = (l1 * l1 - l2 * l2 + L * L) / (2 * L);
    const h = Math.sqrt(Math.max(l1 * l1 - x * x, 0));
    const pp = V.norm(V.sub(pole, V.mul(n, V.dot(pole, n))));
    return [
      V.add(a, V.add(V.mul(n, x), V.mul(pp, h))),
      V.add(a, V.mul(n, L)),
    ];
  }

  const BASE = {
    p: [0, 0, 0],
    hy: 0,
    sy: 0,
    b: 0.1,
    l: 10,
    rw: [0.2, 0.1, 0.35],
    lw: [-0.1, 0.1, 0.35],
    rk: [0.2, 0.5, 0.85],
    ra: [0.18, 0.05, 0],
    la: [-0.18, 0.05, 0],
  };

  const GRIPS = {
    Continental: { ang: 0, label: 'Continental' },
    Eastern: { ang: 35, label: 'Eastern' },
    'Semi-Western': { ang: 65, label: 'Semi-Western' },
  };

  // ===== Esqueleto Anatômico 3D =====
  function joints(o) {
    const hyDeg = clampBio(o.hy, -75, 75);
    const xFactor = clampBio(o.sy - hyDeg, -62, 62);
    const syDeg = hyDeg + xFactor;
    const leanDeg = clampBio(o.l, -22, 38);
    const crouch = clampBio(o.b * 0.85, -0.04, 0.30);

    const pel = [o.p[0], 0.95 - crouch + o.p[1], o.p[2]];
    const hy = rad(hyDeg), sy = rad(syDeg), lr = rad(leanDeg);
    const hv = [Math.cos(hy), 0, -Math.sin(hy)];
    const sv = [Math.cos(sy), 0, -Math.sin(sy)];
    const fw = [Math.sin(hy), 0, Math.cos(hy)];
    const mv = [Math.cos((hy + sy) / 2), 0, -Math.sin((hy + sy) / 2)];

    const M = V.add(pel, [0, 0.26 * Math.cos(lr * 0.5), 0.26 * Math.sin(lr * 0.5)]);
    const neck = V.add(M, [0, 0.3 * Math.cos(lr), 0.3 * Math.sin(lr)]);
    const elR = Math.max(0, o.rw[1] - 0.3) * 0.07;
    const elL = Math.max(0, o.lw[1] - 0.3) * 0.07;

    const SR = V.add(V.add(neck, V.mul(sv, 0.19)), [0, elR, 0]);
    const SL = V.add(V.add(neck, V.mul(sv, -0.19)), [0, elL, 0]);
    const HR = V.add(pel, V.mul(hv, 0.1));
    const HL = V.add(pel, V.mul(hv, -0.1));
    const head = V.add(neck, [0, 0.2 * Math.cos(lr), 0.2 * Math.sin(lr) + 0.02]);

    const fS = [Math.sin(sy), 0, Math.cos(sy)];
    const armPole = (S0, T0, sd) => {
      const w = Math.max(0, Math.min(1, (S0[1] - T0[1]) / 0.58));
      return V.add(
        V.add(V.mul(sv, 0.48 * sd), [0, -0.72 * (1 - w), 0]),
        V.mul(fS, -(0.24 + 0.76 * w))
      );
    };

    const [RE, RW] = ik(SR, V.add(pel, o.rw), 0.3, 0.28, armPole(SR, V.add(pel, o.rw), 1));
    const forearmR = V.norm(V.sub(RW, RE));
    const rawRk = V.norm(o.rk);
    const wristLagOffset = [
      rawRk[0] * 0.92 + forearmR[0] * 0.08,
      rawRk[1] * 0.90 - 0.04,
      rawRk[2] * 0.92 + forearmR[2] * 0.08,
    ];
    const d = clampVecCone(wristLagOffset, forearmR, rad(82));

    let lt = V.add(pel, o.lw);
    if (o.lg > 0) {
      lt = V.add(V.mul(lt, 1 - o.lg), V.mul(V.add(RW, V.mul(d, o.lo)), o.lg));
    }
    const [LE, LW] = ik(SL, lt, 0.3, 0.28, armPole(SL, lt, -1));

    const [RKN, RA] = ik(HR, o.ra, 0.47, 0.47, V.add(fw, V.mul(hv, 0.22)));
    const [LKN, LA] = ik(HL, o.la, 0.47, 0.47, V.add(fw, V.mul(hv, -0.22)));

    const RKH = V.add(RW, V.mul(d, 0.56));
    const RKB = V.add(RW, V.mul(d, 0.3));

    const fy = (side) => {
      const a = hy * 0.3 + side * rad(8);
      return [Math.sin(a), 0, Math.cos(a)];
    };
    const mkFoot = (A, f) => {
      const lift = Math.max(0, A[1] - 0.05);
      const th = Math.min(0.9, lift * 3.5);
      const air = Math.max(0, lift - 0.28) * 0.8;
      const B = [A[0] + f[0] * 0.1, 0.03 + air, A[2] + f[2] * 0.1];
      const hr = 0.15;
      const H = [B[0] - f[0] * hr * Math.cos(th), B[1] + hr * Math.sin(th), B[2] - f[2] * hr * Math.cos(th)];
      const tr = 0.05;
      const T = [
        B[0] + f[0] * tr * Math.cos(th * 0.35),
        Math.max(0.026, B[1] - tr * Math.sin(th * 0.35)),
        B[2] + f[2] * tr * Math.cos(th * 0.35),
      ];
      return { H, B, T };
    };

    const fr = mkFoot(RA, fy(1));
    const fl = mkFoot(LA, fy(-1));

    return {
      pel, neck, head, SR, SL, HR, HL, RE, LE, RW, LW, RKN, LKN, RA, LA, RKH, RKB, M, mv, hv, sv,
      gz: [Math.sin(sy * 0.4), 0, Math.cos(sy * 0.4)],
      RHD: V.add(RW, V.mul(d, 0.05)),
      RTO: fr.T, RBL: fr.B, RHL: fr.H,
      LTO: fl.T, LBL: fl.B, LHL: fl.H,
      lg: o.lg || 0,
      hyDeg, syDeg, xFactor, leanDeg,
      ankleFlexR: Math.max(0, RA[1] - 0.08),
      ankleFlexL: Math.max(0, LA[1] - 0.08),
    };
  }

  // ===== Interpolação Hermite & Marcha =====
  const hm = (p0, p1, m0, m1, h, u) =>
    Array.isArray(p0)
      ? p0.map((_, i) => hm(p0[i], p1[i], m0[i], m1[i], h, u))
      : (2 * u * u * u - 3 * u * u + 1) * p0 +
        (u * u * u - 2 * u * u + u) * h * m0 +
        (-2 * u * u * u + 3 * u * u) * p1 +
        (u * u * u - u * u) * h * m1;

  const tg = (a, b, dt) =>
    Array.isArray(a) ? a.map((_, i) => tg(a[i], b[i], dt)) : (0.72 * (b - a)) / dt;

  function evalPose(stroke, t) {
    if (stroke.hold && t >= stroke.hold) return { ...stroke.keys[stroke.keys.length - 2] };
    const k = stroke.keys, n = k.length;
    let i = n - 1;
    for (let j = 0; j < n; j++) if (k[j].t <= t) i = j;
    const t0 = k[i].t;
    const t1 = i === n - 1 ? stroke.dur : k[i + 1].t;
    const u = Math.min(1, Math.max(0, (t - t0) / (t1 - t0 || 1)));
    const g = (j) => k[(j + n) % n];
    const T = (j) => k[(j + n) % n].t + Math.floor(j / n) * stroke.dur;
    const r = {};
    for (const key in BASE) {
      r[key] = hm(
        g(i)[key],
        g(i + 1)[key],
        tg(g(i - 1)[key], g(i + 1)[key], T(i + 1) - T(i - 1)),
        tg(g(i)[key], g(i + 2)[key], T(i + 2) - T(i)),
        t1 - t0,
        u
      );
    }
    for (const f of ['ra', 'la']) {
      const a = g(i)[f], b = g(i + 1)[f];
      const d = Math.hypot(b[0] - a[0], b[2] - a[2]);
      const o0 = f === 'ra' ? 0.1 : 0.4, sp = f === 'ra' ? 0.55 : 0.6;
      if (d > 0.08) {
        const w = Math.min(1, Math.max(0, (u - o0) / sp));
        const e = w * w * (3 - 2 * w);
        r[f] = [
          a[0] + (b[0] - a[0]) * e,
          Math.max(0.05, a[1] + (b[1] - a[1]) * e) + 0.14 * Math.sin(Math.PI * w) * Math.min(1, d * 2.5),
          a[2] + (b[2] - a[2]) * e,
        ];
      } else if (a[1] < 0.1 && b[1] < 0.1) {
        const e = u * u * (3 - 2 * u);
        r[f] = [a[0] + (b[0] - a[0]) * e, 0.05, a[2] + (b[2] - a[2]) * e];
      }
    }
    return r;
  }

  function gripAt(s, t) {
    if (!s.gr) return [0, 0];
    let sw = 0, sl = 0;
    const E = (a, b, x) => {
      const u = Math.min(1, Math.max(0, (x - a) / (b - a || 1)));
      return u * u * (3 - 2 * u);
    };
    for (const g of s.gr) {
      const w = E(g[0], g[1], t) * (1 - E(g[2], g[3], t));
      sw += w;
      sl += w * g[4];
    }
    return sw > 0 ? [Math.min(1, sw), sl / sw] : [0, 0];
  }

  const RUNS = ['fh', 'bh', 'b1', 'sl', 'rt', 'lb', 'dp', 'rf', 'sa'];
  const SLEN = 0.95;

  function runInfo(s, t, origin = [0, 0]) {
    if (!RUNS.includes(s.id)) return { O: origin, g: 0, ph: 0, dir: [0, 1] };
    const rt = s.id === 'rt';
    const a0 = rt ? 0.55 : 0.42, a1 = s.tc - (rt ? 0.15 : 0.4);
    const b0 = s.tc + 0.45, b1 = Math.min(s.dur - 0.1, s.tc + 1.15);
    const v0 = [origin[0], origin[1] + 1.5];
    const L = Math.hypot(v0[0], v0[1]) || 1e-6;
    const Lm = Math.min(3.2, 4.2 * (a1 - a0));
    const k = L > Lm ? Lm / L : 1;
    const vv = [v0[0] * k, v0[1] * k];
    const D = Math.hypot(vv[0], vv[1]);
    const E = (a, b, x) => sm3(Math.min(1, Math.max(0, (x - a) / (b - a))));
    const F = (x) => E(a0, a1, x) - E(b0, b1, x);
    const f = F(t);
    const sp = (D * Math.abs(F(t + 0.02) - F(t - 0.02))) / 0.04;
    const sg = t >= b0 ? -1 : 1;
    return {
      O: [origin[0] - vv[0] * (1 - f), origin[1] - vv[1] * (1 - f)],
      g: Math.min(1, sp / 0.7),
      ph: (D * (E(a0, a1, t) + E(b0, b1, t))) / SLEN,
      dir: [(sg * vv[0]) / (D || 1), (sg * vv[1]) / (D || 1)],
    };
  }

  function gait(o, R) {
    const d = R.dir, pp = [d[1], -d[0]], B = R.O;
    [['ra', 1, 0], ['la', -1, 0.5]].forEach(([f, sd, po]) => {
      const fr = (R.ph + po) % 1;
      let off, lift = 0;
      if (fr < 0.6) off = SLEN * (0.3 - fr);
      else {
        const u = (fr - 0.6) / 0.4;
        off = SLEN * (-0.3 + 0.6 * sm3(u));
        lift = 0.14 * Math.sin(Math.PI * u);
      }
      const x = B[0] + pp[0] * sd * 0.18 + d[0] * off;
      const z = B[1] + pp[1] * sd * 0.18 + d[1] * off;
      const a = o[f], g = R.g;
      o[f] = [
        a[0] + (x - a[0]) * g,
        Math.max(0.05, a[1] + (0.05 + lift - a[1]) * g),
        a[2] + (z - a[2]) * g,
      ];
    });
    o.p[1] += R.g * 0.03 * Math.abs(Math.sin(Math.PI * 2 * R.ph));
  }

  function fixReach(o) {
    const hy = rad(o.hy), hx = Math.cos(hy) * 0.1, hz = -Math.sin(hy) * 0.1;
    const py = 0.95 - o.b * 0.85 + o.p[1];
    let mx = py;
    [['ra', 1], ['la', -1]].forEach(([f, sg]) => {
      const a = o[f], dx = o.p[0] + sg * hx - a[0], dz = o.p[2] + sg * hz - a[2];
      mx = Math.min(mx, a[1] + Math.sqrt(Math.max(0, 0.93 * 0.93 - dx * dx - dz * dz)));
    });
    if (py > mx) o.b += (py - mx) / 0.85;
    return o;
  }

  const SMW = [0.1, 0.2, 0.4, 0.2, 0.1], SMD = 0.03;

  function poseRaw(s, t, opts = {}) {
    const origin = opts.origin || [0, 0];
    const da = opts.aimDelta || 0;
    const o = evalPose(s, t);
    const R = runInfo(s, t, origin);
    const O = R.O;
    const w = Math.exp(-Math.pow((t - s.tc) / 0.22, 2));
    o.hy += da * 0.45 * w;
    o.sy += da * 0.6 * w;
    o.rk = [o.rk[0] + da * 0.006 * w, o.rk[1], o.rk[2]];
    const g = gripAt(s, t);
    o.lg = g[0];
    o.lo = g[1];
    o.p = [o.p[0] + O[0], o.p[1], o.p[2] + O[1]];
    o.ra = [o.ra[0] + O[0], o.ra[1], o.ra[2] + O[1]];
    o.la = [o.la[0] + O[0], o.la[1], o.la[2] + O[1]];
    if (R.g > 0) gait(o, R);
    return fixReach(o);
  }

  function pose(s, t, opts = {}) {
    const o = poseRaw(s, t, opts);
    if (s.hold && t >= s.hold) return o;
    const X = [];
    for (let k = -3; k <= 3; k++) {
      X.push(poseRaw(s, Math.min(s.dur, Math.max(0, t + k * SMD)), opts));
    }
    const sp = (f, k) =>
      Math.hypot(X[k + 4][f][0] - X[k + 2][f][0], X[k + 4][f][2] - X[k + 2][f][2]) / (2 * SMD);
    const r = { ...o };
    for (const f of ['ra', 'la']) {
      let x = 0, y = 0, z = 0;
      for (let k = -2; k <= 2; k++) {
        const w = SMW[k + 2], a = X[k + 3][f];
        const lift = 0.11 * sstep(0.25, 1.2, sp(f, k));
        x += w * a[0];
        z += w * a[2];
        y += w * Math.max(a[1], 0.05 + lift);
      }
      r[f] = [x, y, z];
    }
    let px = 0, py = 0, pz = 0, b = 0;
    for (let k = -2; k <= 2; k++) {
      const w = SMW[k + 2], a = X[k + 3];
      px += w * a.p[0];
      py += w * a.p[1];
      pz += w * a.p[2];
      b += w * a.b;
    }
    r.p = [px, py, pz];
    r.b = b;
    if (!s.hold && t > s.dur - 0.4) {
      const w = Math.min(1, (t - (s.dur - 0.4)) / 0.4);
      const ws = w * w * (3 - 2 * w);
      const z = poseRaw(s, 0, opts);
      for (const f of ['p', 'ra', 'la', 'rw', 'lw', 'rk']) {
        r[f] = r[f].map((v, i) => v * (1 - ws) + z[f][i] * ws);
      }
      for (const f of ['hy', 'sy', 'l']) r[f] = r[f] * (1 - ws) + z[f] * ws;
      r.b = r.b * (1 - ws) + z.b * ws;
    }
    return fixReach(r);
  }

  // ===== Dinâmica e Cadeia Cinética =====
  function motion(s, t, opts = {}) {
    const e = 0.03;
    const ta = (t - e + s.dur) % s.dur;
    const tb = (t + e) % s.dur;
    const oa = pose(s, ta, opts), ob = pose(s, tb, opts);
    const a = joints(oa), b = joints(ob);
    const sp = (x, y) => V.len(V.sub(y, x)) / (2 * e);
    return {
      hipVelDeg: (ob.hy - oa.hy) / (2 * e),
      shoulderVelDeg: (ob.sy - oa.sy) / (2 * e),
      chain: [
        sp(a.pel, b.pel),
        Math.abs(ob.hy - oa.hy) / (2 * e),
        Math.abs(ob.sy - oa.sy) / (2 * e),
        sp(a.RW, b.RW),
        sp(a.RKH, b.RKH),
      ],
      pelvisVel: V.mul(V.sub(b.pel, a.pel), 1 / (2 * e)),
    };
  }

  // ===== Física Aerodinâmica da Bola (Arrasto + Magnus + Quique) =====
  function sim(c, v, th, az, rpm, full = true) {
    const R = 0.0335, m = 0.057, rho = 1.225, A = Math.PI * R * R;
    const w0 = (rpm * Math.PI) / 30;
    const w = [w0 * az[1], 0, -w0 * az[0]];
    let p = [...c];
    let u = [Math.cos(th) * az[0] * v, Math.sin(th) * v, Math.cos(th) * az[1] * v];
    let t = 0, nb = 0, B = null, N = null, net = 0, pk = c[1];
    const P = [[0, ...p]];
    const dt = 0.002;

    for (let i = 0; i < 2000; i++) {
      const sp = V.len(u) || 1e-6;
      const spinRate = V.len(w);
      const Sr = spinRate > 1 ? (R * spinRate) / sp : 0;
      const Cd = 0.55 + 0.1 * Math.min(1, Sr / 0.4);
      const Cl = spinRate > 1 ? 1 / (2.02 + 0.98 * (sp / (R * spinRate))) : 0;
      const fDrag = -(0.5 * rho * A * Cd * sp) / m;
      const cr = [w[1] * u[2] - w[2] * u[1], w[2] * u[0] - w[0] * u[2], w[0] * u[1] - w[1] * u[0]];
      const clLen = V.len(cr) || 1;
      const fMag = (0.5 * rho * A * Cl * sp * sp) / (m * clLen);
      const a = [
        fDrag * u[0] + fMag * cr[0],
        -9.81 + fDrag * u[1] + fMag * cr[1],
        fDrag * u[2] + fMag * cr[2],
      ];
      u = V.add(u, V.mul(a, dt));
      const q = V.add(p, V.mul(u, dt));

      if (p[2] < 11.885 && q[2] >= 11.885) {
        const fN = (11.885 - p[2]) / (q[2] - p[2] || 1);
        N = p[1] + (q[1] - p[1]) * fN;
        const nx = p[0] + (q[0] - p[0]) * fN;
        if (N < 0.914 + 0.0043 * nx * nx + R) {
          net = 1;
          u = [u[0] * 0.2, 0, -Math.abs(u[2]) * 0.12];
          q[2] = 11.885 - 0.02;
        }
      }

      const p0 = p;
      p = q;
      if (q[1] <= R && u[1] < 0 && p0[1] > R) {
        const fr = (p0[1] - R) / (p0[1] - q[1]);
        p = V.add(p0, V.mul(V.sub(q, p0), fr));
      }
      t += dt;
      pk = Math.max(pk, p[1]);

      if (p[1] <= R + 1e-6 && u[1] < 0) {
        if (!B) B = [...p];
        nb++;
        const vyIn = -u[1];
        const corY = Math.max(0.68, 0.78 - 0.004 * vyIn);
        const slipX = u[0] + w[2] * R;
        const slipZ = u[2] - w[0] * R;
        const slipSp = Math.hypot(slipX, slipZ);
        const mu = 0.55;
        const maxImpulse = mu * (1 + corY) * vyIn;
        if (slipSp > 1e-4) {
          const rollImpulse = 0.4 * slipSp;
          const J = Math.min(maxImpulse, rollImpulse);
          const dirX = slipX / slipSp, dirZ = slipZ / slipSp;
          u[0] -= J * dirX;
          u[2] -= J * dirZ;
          const dW = J / (0.62 * R);
          w[2] -= dW * dirX;
          w[0] += dW * dirZ;
        }
        u[1] = vyIn * corY;
        p[1] = R + 1e-4;
      }

      if (i % 6 === 0 || (nb > 0 && i % 3 === 0)) P.push([t, ...p]);
      if (!full && nb >= 1) break;
      if (nb >= 2 || t > 3.2) break;
    }
    return { P, B, N, net, pk, T: t };
  }

  function solve(c, tx, tz, v, rpm) {
    const dx = tx - c[0], dz = tz - c[2];
    const D = Math.hypot(dx, dz) || 1;
    const az = [dx / D, dz / D];
    let lo = -0.35, hi = 0.7, m = 0;
    for (let i = 0; i < 26; i++) {
      m = (lo + hi) / 2;
      const r = sim(c, v, m, az, rpm, false);
      const d = r.B ? Math.hypot(r.B[0] - c[0], r.B[2] - c[2]) : 1e3;
      if (d > D) hi = m;
      else lo = m;
    }
    const r = sim(c, v, m, az, rpm, true);
    r.th = (m * 180) / Math.PI;
    return r;
  }

  // ===== Camada de Comparação com Vídeo Real =====
  function analyzePoseComparison(stroke, t, observed = {}) {
    const p = pose(stroke, t);
    const j = joints(p);
    const diffs = {};
    if (typeof observed.hipYaw === 'number') diffs.hipYaw = +(observed.hipYaw - j.hyDeg).toFixed(1);
    if (typeof observed.shoulderYaw === 'number') diffs.shoulderYaw = +(observed.shoulderYaw - j.syDeg).toFixed(1);
    if (typeof observed.xFactor === 'number') diffs.xFactor = +(observed.xFactor - j.xFactor).toFixed(1);
    if (typeof observed.trunkLean === 'number') diffs.trunkLean = +(observed.trunkLean - j.leanDeg).toFixed(1);
    return {
      phase: t < stroke.tc ? 'Preparação / Aceleração' : Math.abs(t - stroke.tc) < 0.04 ? 'Contato' : 'Finalização',
      timeToContactMs: Math.round((t - stroke.tc) * 1000),
      reference: {
        hipYaw: +j.hyDeg.toFixed(1),
        shoulderYaw: +j.syDeg.toFixed(1),
        xFactor: +j.xFactor.toFixed(1),
        trunkLean: +j.leanDeg.toFixed(1),
      },
      diffs,
      joints: j,
    };
  }

  global.TenisLabModel = {
    version: 'Me23-Core-1.0',
    rad, deg, V, rotAroundAxis, ik, BASE, GRIPS,
    joints, evalPose, poseRaw, pose, motion,
    sim, solve, analyzePoseComparison,
    STROKES: {}, // Preenchido pelo banco S de golpes
    SHOTS: [],   // Preenchido pela tabela SH
  };
})(typeof window !== 'undefined' ? window : globalThis);
