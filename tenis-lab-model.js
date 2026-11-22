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

  const K = (t, o) => ({ t, ...BASE, ...o });

  const GRIPS = {
    Continental: { ang: 0, label: 'Continental' },
    Eastern: { ang: 35, label: 'Eastern' },
    'Semi-Western': { ang: 65, label: 'Semi-Western' },
  };

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

  // ===== Banco de Golpes (S) =====
const S={
fh:{n:'Forehand',dur:2.4,tc:1.25,inb:[.3,.8,4],intro:'Forehand: o golpe de direita. Observe o giro do tronco, a raquete caindo atrás da bola e a finalização sobre o ombro oposto.',
keys:[K(0,{}),K(.5,{hy:38,sy:92,b:.14,l:12,p:[.05,0,-.12],rw:[.4,.2,-.35],lw:[-.3,.4,.4],rk:[.1,.7,-.7],ra:[.25,.05,-.2],la:[-.2,.05,.3]}),K(.95,{hy:25,sy:72,b:.2,l:15,rw:[.35,-.1,-.4],lw:[-.35,.3,.35],rk:[.1,-.4,-.9],p:[.05,0,-.1],ra:[.25,.05,-.2],la:[-.2,.05,.3]}),K(1.1,{hy:20,sy:66,b:.16,l:15,rw:[.35,-.05,-.2],lw:[-.38,.3,.25],rk:[.1,-.2,-.9],p:[0,0,.12],ra:[.24,.05,-.18],la:[-.2,.05,.32]}),K(1.18,{hy:-7,sy:33,b:.14,l:15,rw:[.35,.05,.2],lw:[-.4,.3,.15],rk:[.1,-.21,-.8],p:[0,0,.16],ra:[.23,.05,-.16],la:[-.2,.05,.34]}),K(1.25,{hy:-16,sy:1,b:.12,l:15,rw:[.35,.12,.5],lw:[-.4,.3,.1],rk:[.1,.15,1],p:[0,0,.2],ra:[.22,.05,-.15],la:[-.2,.05,.35]}),K(1.3,{hy:-17,sy:1,b:.1,l:14,rw:[.3,.35,.75],lw:[-.45,.25,0],rk:[-.22,.54,.63],p:[0,0,.22],ra:[.2,.05,-.1],la:[-.2,.05,.36]}),K(1.6,{hy:-32,sy:-52,b:.04,l:8,rw:[-.2,.55,.3],lw:[-.45,.15,-.1],rk:[-.5,-.3,-.6],p:[0,0,.3],ra:[.1,.2,.15],la:[-.2,.05,.4]}),K(2,{hy:-5,sy:-15,rw:[.1,.2,.4],lw:[-.1,.15,.35],rk:[.2,.6,.8],p:[0,0,.1],ra:[.15,.05,.1],la:[-.15,.05,.2]})],
tips:[[0,['HR','HL'],'Posição de espera','Pés afastados, joelhos flexionados e raquete à frente. Peso na ponta dos pés, pronto para reagir.'],[.5,['SR','SL','HR','HL'],'Unit turn (bloco único)','Tronco, ombros e braço giram juntos, assim que a bola sai da raquete do rival. Ombros ~90° e quadril só 30–45°: essa diferença guarda a energia elástica.'],[.95,['RKH','RKN','LKN'],'Queda da raquete','A raquete cai abaixo da bola enquanto os joelhos flexionam. Essa é a fonte da potência e do topspin.'],[1.18,['HR','HL','SR','SL'],'Pelve → tronco (−75 ms)','A pelve atinge o pico de rotação ~75 ms antes do contato e desacelera, passando o momento ao tronco (pico ~57 ms antes). Depois vêm o ombro (+24 ms) e o cotovelo (+35 ms). O timing, mais que a força, separa elite de jovens de alto nível.'],[1.25,['RW','RKH'],'Ponto de contato','Bola na frente do corpo, na altura da cintura. O quadril abre primeiro, depois ombro e braço. Punho firme.'],[1.6,['SL','RW'],'Finalização','A raquete termina sobre o ombro oposto, com o corpo equilibrado e o olhar na zona de contato.'],[2,['RA','LA'],'Recuperação','Volte rápido à posição de espera, com passos curtos, e prepare a próxima bola.']]},
bh:{n:'Backhand 2 mãos',dur:2.4,tc:1.25,inb:[-.3,.8,4],intro:'Backhand de duas mãos. A mão esquerda comanda o golpe, e o ombro direito aponta para a bola no giro.',
keys:[K(0,{}),K(.5,{hy:-50,sy:-85,l:12,p:[0,0,-.1],rw:[-.3,.2,-.2],lw:[-.35,.25,-.05],rk:[-.5,.7,-.5],ra:[.2,.05,.25],la:[-.25,.05,-.15]}),K(.95,{hy:-30,sy:-80,b:.2,p:[0,0,-.1],l:15,rw:[-.25,0,-.4],lw:[-.35,.05,-.3],rk:[-.3,-.3,-.9],ra:[.2,.05,.25],la:[-.25,.05,-.15]}),K(1.1,{hy:-20,sy:-66,b:.16,l:15,rw:[-.25,0,-.15],lw:[-.2,.1,-.05],rk:[-.2,-.2,-.9],p:[0,0,.12],ra:[.2,.05,.28],la:[-.25,.05,-.12]}),K(1.18,{hy:7,sy:-33,b:.14,l:15,rw:[-.25,.08,.25],lw:[-.15,.1,.2],rk:[-.1,-.21,-.8],p:[0,0,.16],ra:[.2,.05,.29],la:[-.25,.05,-.11]}),K(1.25,{hy:16,sy:-1,b:.12,l:15,rw:[-.1,.12,.55],lw:[-.05,.12,.5],rk:[0,.15,1],p:[0,0,.2],ra:[.2,.05,.3],la:[-.25,.05,-.1]}),K(1.3,{hy:17,sy:-1,b:.1,l:14,rw:[-.1,.4,.7],lw:[0,.35,.55],rk:[.22,.54,.63],p:[0,0,.22],ra:[.2,.05,.34],la:[-.25,.05,-.1]}),K(1.6,{hy:32,sy:52,b:.04,l:8,rw:[.3,.55,.3],lw:[.1,.5,.3],rk:[.5,.3,-.6],p:[0,0,.3],ra:[.2,.05,.4],la:[-.2,.15,.1]}),K(2,{hy:5,sy:10,rw:[.1,.2,.4],lw:[-.1,.15,.35],rk:[.2,.6,.8],p:[0,0,.1],ra:[.15,.05,.2],la:[-.15,.05,.1]})],
tips:[[0,['HR','HL'],'Posição de espera','Base firme e raquete à frente. A empunhadura de duas mãos já fica pronta.'],[.5,['SR','LW','RW'],'Giro lateral','O ombro direito gira em direção à bola, as duas mãos levam a raquete para trás junto ao corpo.'],[.95,['RKH','LKN','RKN'],'Preparação baixa','Flexione os joelhos e deixe a raquete abaixo da bola. Mantenha as mãos próximas do tronco.'],[1.25,['LW','RW'],'Contato','Contato na frente do corpo. A mão esquerda conduz, o corpo desenrola do quadril para os ombros.'],[1.6,['SR','SL'],'Finalização alta','Termine com a raquete sobre o ombro, os braços alongados e o corpo equilibrado.'],[2,['RA','LA'],'Recuperação','Volte ao centro da quadra rapidamente.']]},
sv:{n:'Saque',dur:3.2,tc:1.7,inb:[0,-1,0],intro:'Saque. Sequência: base firme no solo, lançamento cinemático do toss com braço estendido, flexão de joelhos para troféu, queda da raquete, salto e impacto no ponto mais alto.',
keys:[K(0,{sy:55,hy:45,b:.06,rw:[.15,.1,.3],lw:[-.22,-.32,.25],rk:[.1,-.2,.7],ra:[.25,.05,-.25],la:[-.25,.05,.15]}),K(.4,{sy:65,hy:52,b:.08,rw:[.25,.05,.1],lw:[-.18,.18,.35],rk:[.15,-.4,.4],ra:[.25,.05,-.25],la:[-.25,.05,.15]}),K(.85,{sy:78,hy:58,b:.18,rw:[.32,.25,-.05],lw:[-.12,.82,.42],rk:[.05,.2,-.3],ra:[.25,.05,-.25],la:[-.25,.05,.15]}),K(1.22,{sy:82,hy:62,b:.32,l:-6,rw:[.35,.52,-.18],lw:[-.08,1.15,.48],rk:[0,.75,-.75],ra:[.25,.05,-.25],la:[-.25,.05,.15]}),K(1.52,{sy:52,hy:42,b:.10,l:14,p:[0,.12,.12],rw:[.3,.42,-.25],lw:[-.12,.42,.4],rk:[0,.2,-.8],ra:[.25,.16,-.18],la:[-.25,.18,.22]}),K(1.62,{sy:30,hy:30,b:.04,l:12,p:[0,.14,.2],rw:[.34,.78,-.1],lw:[-.12,.38,.38],rk:[0,.55,-.6],ra:[.25,.2,-.12],la:[-.25,.22,.28]}),K(1.7,{sy:10,hy:15,b:-.03,l:10,p:[0,.16,.28],rw:[.19,1.17,.2],lw:[-.3,.32,.28],rk:[0,1,.25],ra:[.24,.24,-.05],la:[-.25,.26,.34]}),K(1.78,{sy:9,hy:9,b:.08,l:25,p:[0,.10,.38],rw:[-.1,.6,.55],lw:[-.35,.2,.2],rk:[-.1,-.56,.72],ra:[.22,.22,.08],la:[-.25,.12,.45]}),K(2.1,{sy:-55,hy:-35,b:.12,l:32,p:[0,0,.55],rw:[-.3,.1,.5],lw:[-.4,.1,.1],rk:[-.3,-.7,.5],ra:[.20,.26,.12],la:[-.25,.05,.52]}),K(2.7,{sy:-10,b:.08,p:[0,0,.55],ra:[.18,.05,.62],la:[-.22,.05,.48]})],
tips:[[0,['HR','HL'],'Postura inicial','Pés na largura dos ombros, corpo de lado para a rede, peso distribuído e ritmo calmo.'],[.6,['LW','SL'],'Lançamento','O braço esquerdo sobe reto e solta a bola à frente, um pouco acima do alcance da raquete.'],[1.1,['RW','RE','LKN','RKN'],'Posição de troféu','Joelhos flexionados, cotovelo alto e braço esquerdo apontando para a bola. É aqui que se armazena a energia.'],[1.4,['RKH','RE'],'Queda da raquete','A raquete cai atrás das costas, como um chicote, enquanto as pernas começam a empurrar.'],[1.7,['RW','RKH','RA','LA'],'Contato no ponto alto','Rotação interna do ombro (~41%) e flexão palmar do punho (~32%) geram a maior parte da velocidade da raquete. A pronção visível do antebraço é consequência da rotação interna, não um motor independente. Braço estendido e contato no ponto mais alto.'],[2.1,['SL','RA'],'Finalização','O braço cruza o corpo, e o pé de trás entra na quadra para você já ficar pronto.'],[2.7,['RA','LA'],'Recuperação','Equilibre-se e volte à posição de espera.']]},
b1:{n:'Backhand 1 mão',dur:2.4,tc:1.25,inb:[-.3,.8,4],intro:'Backhand de uma mão. O braço esquerdo se estende para trás como contrapeso, e o braço da raquete fica alongado no contato.',
keys:[K(0,{}),K(.5,{hy:-50,sy:-85,l:12,p:[0,0,-.1],rw:[-.25,.2,-.2],lw:[-.15,.25,.15],rk:[-.5,.6,-.5],ra:[.2,.05,.25],la:[-.25,.05,-.15]}),K(.95,{hy:-30,sy:-85,b:.2,p:[0,0,-.1],rw:[-.2,0,-.4],lw:[-.2,.15,-.2],rk:[-.3,-.3,-.9],ra:[.2,.05,.25],la:[-.25,.05,-.15]}),K(1.1,{hy:-20,sy:-66,b:.16,l:15,rw:[-.25,0,-.15],lw:[-.2,.2,-.3],rk:[-.2,-.2,-.9],p:[0,0,.12],ra:[.2,.05,.28],la:[-.25,.05,-.12]}),K(1.18,{hy:7,sy:-33,b:.14,l:15,rw:[-.28,.08,.3],lw:[-.2,.25,-.4],rk:[-.1,-.21,-.8],p:[0,0,.16],ra:[.2,.05,.29],la:[-.25,.05,-.11]}),K(1.25,{hy:16,sy:-1,b:.12,l:15,p:[0,0,.2],rw:[-.3,.12,.6],lw:[-.2,.3,-.45],rk:[-.1,.15,1],ra:[.2,.05,.3],la:[-.25,.05,-.1]}),K(1.3,{hy:17,sy:-1,b:.1,l:14,rw:[-.1,.4,.7],lw:[0,.3,-.45],rk:[.22,.54,.63],p:[0,0,.22],ra:[.2,.05,.34],la:[-.25,.05,-.1]}),K(1.6,{hy:32,sy:40,b:.04,l:8,p:[0,0,.3],rw:[.05,.45,.45],lw:[0,.3,-.4],rk:[.3,.4,.3],ra:[.2,.05,.4],la:[-.2,.15,.1]}),K(2,{hy:5,sy:10,rw:[.1,.2,.4],lw:[-.1,.15,.35],rk:[.2,.6,.8],p:[0,0,.1],ra:[.15,.05,.2],la:[-.15,.05,.1]})],
tips:[[0,['HR','HL'],'Espera','Base estável, raquete à frente com a mão esquerda no pescoço dela.'],[.5,['SR','SL','LW'],'Giro de ombros','Ombro direito aponta para a bola e o peso vai para a perna de trás.'],[.95,['HR','HL','RKN'],'Quadril lidera','O quadril começa a abrir primeiro, enquanto os ombros ainda ficam fechados. Essa separação é a energia do golpe.'],[1.25,['RW','LW','RKH'],'Contato alongado','Braço da raquete estendido à frente. O braço esquerdo vai para trás em equilíbrio, e o peso passa para a perna da frente.'],[1.6,['SR','RW'],'Finalização','A raquete sobe e se alonga, com o tronco ereto e olhar no contato.'],[2,['RA','LA'],'Recuperação','Volte ao centro e prepare o próximo golpe.']]},
sl:{n:'Slice',dur:2.6,tc:1.3,inb:[-.3,.7,3.5],intro:'Slice de backhand. Raquete alta, descida suave de cima para baixo com face levemente aberta, e a bola ganha efeito de backspin.',
keys:[K(0,{}),K(.55,{hy:-45,sy:-80,b:.1,p:[0,0,-.1],rw:[-.2,.6,-.25],lw:[-.15,.3,.1],rk:[-.2,.9,-.3],ra:[.2,.05,.25],la:[-.25,.05,-.15]}),K(1,{hy:-20,sy:-68,b:.18,p:[0,0,.05],rw:[-.25,.35,.2],lw:[-.2,.3,-.35],rk:[-.2,.4,.8],ra:[.2,.05,.28],la:[-.25,.05,-.15]}),K(1.12,{hy:-18,sy:-62,b:.2,l:16,p:[0,0,.1],rw:[-.22,.5,.05],lw:[-.2,.3,-.4],rk:[-.2,.7,-.35],ra:[.2,.05,.28],la:[-.25,.05,-.14]}),K(1.22,{hy:4,sy:-40,b:.2,l:17,p:[0,0,.16],rw:[-.25,.32,.4],lw:[-.2,.28,-.46],rk:[-.12,.45,.4],ra:[.2,.05,.29],la:[-.25,.05,-.13]}),K(1.3,{hy:14,sy:-4,b:.2,l:18,p:[0,0,.2],rw:[-.25,.2,.6],lw:[-.2,.25,-.5],rk:[-.05,.2,1],ra:[.2,.05,.3],la:[-.25,.05,-.12]}),K(1.38,{hy:15,sy:-2,b:.19,l:17,p:[0,0,.24],rw:[-.2,.1,.75],lw:[-.2,.28,-.5],rk:[-.04,-.05,1],ra:[.2,.05,.32],la:[-.25,.05,-.11]}),K(1.75,{hy:5,sy:-10,b:.18,p:[0,0,.28],rw:[-.1,0,.85],lw:[-.2,.3,-.55],rk:[0,-.3,1],ra:[.2,.05,.34],la:[-.25,.05,-.1]}),K(2.2,{hy:3,sy:0,rw:[.1,.2,.4],lw:[-.1,.15,.35],rk:[.2,.6,.8],p:[0,0,.1],ra:[.15,.05,.2],la:[-.15,.05,.1]})],
tips:[[0,['HR','HL'],'Espera','Posição neutra, pronto para ler a bola.'],[.55,['RW','RKH'],'Raquete alta','A raquete sobe acima da altura da bola com o cotovelo flexionado e o ombro fechado.'],[1,['RW','LW'],'Descida controlada','O braço desce acompanhando o peso que avança para a perna da frente. Não force a velocidade.'],[1.3,['RW','RKH','RA'],'Contato','Face da raquete levemente aberta, punho firme, contato na frente do corpo. Isso gera o backspin.'],[1.75,['RW','RKH'],'Acompanhamento','A raquete continua para frente e para baixo, e o tronco permanece estável.'],[2.2,['RA','LA'],'Recuperação','Retorne rápido à base.']]},
sm:{n:'Smash',dur:2.8,tc:1.3,inb:[.2,1.5,1.5],intro:'Smash. Passos laterais para trás, braço esquerdo apontando para a bola e um golpe forte, parecido com o saque.',
keys:[K(0,{}),K(.35,{sy:55,hy:45,b:.12,p:[0,0,-.15],rw:[.32,.38,-.3],lw:[-.2,1.12,.4],rk:[.1,.6,-.6],ra:[.25,.05,-.28],la:[-.25,.05,.08]}),K(.8,{sy:75,hy:55,b:.24,l:-4,p:[0,0,-.2],rw:[.34,.38,-.3],lw:[-.1,1.14,.45],rk:[0,.6,-.7],ra:[.25,.05,-.28],la:[-.25,.05,.08]}),K(1.05,{sy:70,hy:50,b:.24,p:[0,0,-.15],rw:[.28,.3,-.35],lw:[-.2,.95,.4],rk:[0,-.7,-.5],ra:[.25,.05,-.28],la:[-.25,.05,.08]}),K(1.17,{sy:48,hy:38,b:.08,l:12,p:[0,.09,-.05],rw:[.32,.7,-.12],lw:[-.15,.42,.4],rk:[0,.14,-.8],ra:[.25,.18,-.22],la:[-.25,.18,.12]}),K(1.3,{sy:10,hy:10,b:-.02,l:10,p:[0,.12,.1],rw:[.19,1.17,.2],lw:[-.3,.3,.3],rk:[0,1,.2],ra:[.22,.24,-.06],la:[-.25,.24,.26]}),K(1.38,{sy:9,hy:9,b:.08,l:25,p:[0,.08,.2],rw:[-.1,.6,.55],lw:[-.35,.2,.2],rk:[-.1,-.56,.72],ra:[.2,.18,.04],la:[-.25,.12,.32]}),K(1.7,{sy:-50,hy:-30,l:32,b:.12,p:[0,0,.38],rw:[-.3,.1,.5],lw:[-.4,.1,.1],rk:[.3,-.7,.5],ra:[.18,.22,.08],la:[-.25,.05,.38]}),K(2.2,{b:.08,p:[0,0,.38],ra:[.16,.05,.44],la:[-.20,.05,.34]})],
tips:[[0,['HR','HL'],'Leitura da bola','Observe a bola alta e prepare os passos laterais.'],[.35,['LW','RKH'],'Passos para trás','Gire de lado, com a raquete já alta e o braço esquerdo apontando para a bola.'],[.8,['RW','RE','LKN','RKN'],'Posição de troféu','Joelhos flexionados, cotovelo alto. O peso fica na perna de trás.'],[1.05,['RKH','RE'],'Queda da raquete','A raquete cai nas costas em chicote, e as pernas começam a empurrar.'],[1.3,['RW','RKH','RA','LA'],'Contato alto','Braço estendido, salto curto, contato no ponto mais alto. A cadeia sobe das pernas até a raquete.'],[1.7,['SL','RA'],'Finalização','O braço cruza o corpo, e o peso termina na perna da frente.'],[2.2,['RA','LA'],'Recuperação','Retorne ao centro da quadra.']]},
vl:{n:'Voleio',dur:1.6,tc:.65,inb:[.2,.5,3],intro:'Voleio. Movimento curto, sem swing amplo: apenas o bloqueio com punho firme, dando um passo à frente.',
keys:[K(0,{}),K(.35,{sy:50,hy:35,b:.2,l:15,rw:[.35,.35,.1],lw:[-.25,.3,.3],rk:[.1,.8,.5],ra:[.25,.05,-.05],la:[-.2,.05,.3]}),K(.55,{sy:38,hy:28,b:.23,l:18,p:[0,0,.18],rw:[.35,.3,.3],lw:[-.3,.3,.25],rk:[.1,.55,.45],ra:[.25,.05,-.05],la:[-.2,.05,.35]}),K(.65,{sy:12,hy:10,b:.25,l:20,p:[0,0,.25],rw:[.35,.25,.55],lw:[-.35,.3,.2],rk:[.05,.3,1],ra:[.25,.05,-.05],la:[-.2,.05,.4]}),K(.72,{sy:10,hy:8,b:.23,l:20,p:[0,0,.25],rw:[.33,.25,.6],lw:[-.32,.3,.25],rk:[.03,.2,1],ra:[.25,.05,-.05],la:[-.2,.05,.4]}),K(.95,{sy:20,hy:15,b:.22,l:20,p:[0,0,.25],rw:[.3,.25,.62],lw:[-.3,.3,.3],rk:[.05,.15,1],ra:[.25,.05,-.05],la:[-.2,.05,.4]}),K(1.3,{sy:5,rw:[.2,.15,.4],lw:[-.1,.1,.35],p:[0,0,.1],ra:[.2,.05,.05],la:[-.18,.05,.15]})],
tips:[[0,['HR','HL'],'Raquete à frente','Raquete sempre à frente do corpo e na altura dos olhos, pronta para o bloqueio.'],[.35,['SR','RW'],'Preparação curta','Gire apenas os ombros. A raquete não passa da linha do corpo.'],[.65,['RW','RKH'],'Bloqueio','Passo à frente e punho firme. A bola é bloqueada na frente do corpo, sem swing amplo.'],[.95,['RW','LA'],'Acompanhamento','Acompanhe a bola com a raquete por poucos centímetros e mantenha o equilíbrio.'],[1.3,['RA','LA'],'Reposicionamento','Recue a raquete à frente e prepare o próximo voleio.']]}};
function addSplit(s){const D=.28,d=s.keys[1].hy<0?-1:1;s.keys.shift();s.tips.shift();s.tl=.4;s.keys.forEach(k=>k.t+=D);s.tips.forEach(a=>a[0]+=D);s.tc+=D;s.dur+=D;s.keys.unshift(K(0,{}),K(.22,{p:[0,.08,0],b:.02,ra:[.2,.15,0],la:[-.2,.15,0]}),K(.4,{b:.24,l:14,ra:[.26,.05,0],la:[-.26,.05,0]}),K(.56,{b:.2,l:8,hy:12*d,sy:35*d,p:[.12*d,0,-.03],rw:[.3*d,.2,.1],lw:[0,.25,.3],rk:[.15*d,.6,.5],ra:d>0?[.38,.05,-.04]:[.14,.05,-.02],la:d>0?[-.14,.05,-.02]:[-.38,.05,-.04]}));s.tips.unshift([.42,['LKN','RKN'],'Arrancada + unit turn','No pouso, a perna de fora empurra o chao e o primeiro passo ja vai para o lado da bola, enquanto quadril e ombros comecam a girar juntos.'],[0,['RA','LA'],'Split step','Pequeno salto no instante em que o rival bate na bola. Aterrisse com base larga e joelhos flexionados, pronto para sair em qualquer direção.'])}
S.ap={n:'Aproximação',dur:3.4,tc:1.2,inb:[.3,.8,4],intro:'Aproximação: unit turn em movimento, passo à frente, forehand profundo na paralela, corrida até a rede e split step antes do voleio.',
keys:[K(0,{p:[0,0,-.6],ra:[.2,.05,-.6],la:[-.2,.05,-.6]}),K(.4,{hy:30,sy:80,b:.15,l:12,p:[0,0,-.3],rw:[.4,.2,-.3],lw:[-.2,.4,.4],rk:[.1,.7,-.7],ra:[.25,.05,-.5],la:[-.2,.05,0]}),K(.8,{hy:25,sy:70,b:.2,l:15,p:[0,0,.1],rw:[.35,-.05,-.4],lw:[-.35,.3,.35],rk:[.1,-.4,-.9],ra:[.25,.05,-.3],la:[-.2,.05,.5]}),K(1.2,{hy:0,sy:20,b:.12,l:15,p:[0,0,.5],rw:[.35,.15,.5],lw:[-.4,.3,.1],rk:[.1,.15,1],ra:[.2,.05,.1],la:[-.2,.05,.55]}),K(1.6,{hy:-25,sy:-50,b:.06,l:10,p:[0,0,.9],rw:[-.2,.5,.3],lw:[-.45,.15,-.1],rk:[-.5,-.3,-.6],ra:[.15,.05,.7],la:[-.2,.05,.6]}),K(2.2,{b:.1,p:[0,0,2],ra:[.2,.05,1.6],la:[-.2,.05,2.2]}),K(2.7,{p:[0,.07,2.6],b:.05,ra:[.2,.2,2.3],la:[-.2,.2,2.5]}),K(3,{b:.22,p:[0,0,2.6],rw:[.2,.2,.4],lw:[-.2,.2,.4],rk:[.1,.8,.5],ra:[.25,.05,2.4],la:[-.25,.05,2.5]})],
tips:[[0,['HR','HL'],'Leitura','Observe a bola curta e comece o deslocamento já com a raquete à frente.'],[.4,['SR','SL','HR','HL'],'Unit turn em movimento','Gire o tronco como um bloco enquanto corre. Não espere chegar para preparar.'],[.8,['LA','RKN'],'Passo à frente','O pé da frente se planta antes do golpe: o peso avança junto com o corpo.'],[1.2,['RW','RKH'],'Contato na aproximação','Bola funda na paralela encurta seu caminho até a rede e cobre a passada reta.'],[1.6,['RA','SL'],'Acompanhar a bola','Siga a direção da bola para a rede e continue em frente, sem parar.'],[2.7,['RA','LA'],'Split step na rede','Pare com pequeno salto quando o rival vai bater e aterrisse equilibrado.'],[3,['RW','RKH'],'Pronto para voleiar','Raquete à frente e alta, base larga e peso na ponta dos pés.']]};
['fh','bh','b1','sl'].forEach(k=>addSplit(S[k]));S.ap.keys.push({...S.ap.keys[S.ap.keys.length-1],t:3.2});S.ap.hold=3;
const cl=(s,n,intro,mods,tips)=>{const c=JSON.parse(JSON.stringify(s));c.n=n;c.intro=intro;c.tips=tips;c.keys.forEach(k=>{const m=mods.find(m=>Math.abs(m[0]-k.t)<.011);if(m)Object.assign(k,m[1])});return c};
S.kk=cl(S.sv,'Saque kick','Saque kick: toss atrás da cabeça e levemente à esquerda, mais arco nas costas e a raquete subindo de baixo para cima, escovando a bola para gerar topspin.',[[1.22,{l:-14,lw:[-.12,1.15,.22],rk:[0,.4,-.8]}],[1.52,{rk:[0,-1,-.1],rw:[.2,.3,-.35]}],[1.7,{l:0,p:[0,.14,.12],rw:[.1,1.17,.12],rk:[-.35,1,.1]}],[1.78,{rk:[-.3,-.45,.7]}]],[[0,['HR','HL'],'Postura','Pegada continental, base estável e ritmo calmo. O kick sai do mesmo setup do saque plano.'],[.6,['LW','SL'],'Toss atrás da cabeça','A bola sobe um pouco atrás da cabeça e levemente à esquerda (para destros). Quanto mais atrás, mais íngreme a subida da raquete e mais topspin.'],[1.1,['SR','LKN','RKN'],'Arco das costas','Joelhos flexionados e mais arco que no saque plano. É exigente: o kick sobrecarrega mais as costas e o ombro.'],[1.4,['RKH','RE'],'Queda da raquete','A raquete cai atrás e o cotovelo fica para trás por mais tempo: a raquete sobe antes do cotovelo.'],[1.7,['RW','RKH'],'Escovar de baixo para cima','A raquete sobe pela parte de trás da bola, de 7h para 1h no relógio. A velocidade da raquete segue alta: é um golpe rápido redirecionado em spin.'],[2.1,['SL','RA'],'Finalização','O braço cruza o corpo e o pé da frente aterrissa dentro da quadra.'],[2.7,['RA','LA'],'Recuperação','Equilibre-se e volte à posição inicial.']]);S.kk.inb=[.15,-1,-.35];
S.rt={n:'Devolução',dur:2.2,tc:1,tl:.42,inb:[.3,.8,4],intro:'Devolução de saque: split step no contato do sacador, giro curto com backswing de cerca de 2/3 e contato à frente com o corpo indo para frente.',
keys:[K(0,{b:.14,l:14,ra:[.25,.05,0],la:[-.25,.05,0]}),K(.35,{p:[0,.08,0],b:.02,ra:[.2,.15,0],la:[-.2,.15,0]}),K(.55,{b:.26,l:16,ra:[.27,.05,0],la:[-.27,.05,0]}),K(.72,{hy:25,sy:66,b:.2,p:[.1,0,-.02],rw:[.35,.2,0],lw:[-.05,.25,.3],rk:[.12,.55,.3],ra:[.38,.05,-.03],la:[-.15,.05,-.02]}),K(.9,{hy:7,sy:58,b:.18,p:[.1,0,.08],rw:[.35,.08,.2],lw:[-.35,.28,.2],rk:[.1,.09,.35],ra:[.34,.05,-.05],la:[-.2,.05,.2]}),K(.95,{hy:-10,sy:30,b:.16,p:[.1,0,.12],rw:[.35,.1,.35],lw:[-.38,.3,.15],rk:[.1,0,.6],ra:[.32,.05,-.05],la:[-.2,.05,.25]}),K(1,{hy:-7,sy:-4,b:.14,p:[.1,0,.15],rw:[.35,.12,.5],lw:[-.4,.3,.1],rk:[.1,.15,1],ra:[.3,.05,-.05],la:[-.2,.05,.3]}),K(1.08,{hy:-14,sy:2,b:.14,p:[.1,0,.18],rw:[.3,.3,.68],lw:[-.45,.25,0],rk:[-.2,.45,.7],ra:[.3,.05,-.05],la:[-.2,.05,.32]}),K(1.4,{hy:-25,sy:-35,b:.08,p:[.1,0,.3],rw:[-.1,.45,.4],lw:[-.4,.2,0],rk:[-.3,.2,.5],ra:[.25,.05,.1],la:[-.2,.05,.34]}),K(1.8,{b:.12,p:[0,0,.1],ra:[.2,.05,0],la:[-.2,.05,.1]})],
tips:[[0,['HR','HL'],'Posição','Cerca de um metro atrás da linha, joelhos flexionados, raquete à frente e um pequeno passo à frente enquanto o sacador arma.'],[.55,['RA','LA'],'Split step no contato do sacador','Pouse no instante em que o sacador bate na bola. Você ainda não sabe a direção, então a pré-carga permite sair para qualquer lado.'],[.72,['SR','SL','HR'],'Giro curto','Passo com a perna de fora e giro do quadril, que já cria o backswing. Backswing de cerca de 2/3 do normal: quanto maior o saque, menor o backswing.'],[1,['RW','RKH'],'Contato à frente','Contato à frente do centro de gravidade, com o corpo ainda indo para frente. Aproveite a velocidade do saque.'],[1.4,['SL','RA'],'Finalização curta','Termine curto e volte ao centro, pronto para a terceira bola.']]};
{const T=.27,B=-.05,r=(s,a)=>{s.gr=a};
r(S.fh,[[-1,-.5,1.1,1.36,T],[2.3,2.55,9,10,T]]);
r(S.bh,[[-1,-.5,.5,.8,T],[.5,.8,1.95,2.2,B],[2.3,2.55,9,10,T]]);
r(S.b1,[[-1,-.5,1.05,1.35,T],[2.3,2.55,9,10,T]]);
r(S.sl,[[-1,-.5,1.1,1.42,T],[2.5,2.75,9,10,T]]);
r(S.rt,[[-1,-.5,.75,.98,T],[1.7,1.95,9,10,T]]);
r(S.ap,[[-1,-.5,.55,.95,T],[2.7,2.95,9,10,T]]);
r(S.vl,[[-1,-.5,.38,.62,T],[1,1.2,9,10,T]]);
S.rt.db=5.8}
S.lb=cl(S.fh,'Lob','Lob de ataque: começa como um forehand (disfarce), mas a raquete cai mais baixo e sobe mais íngreme, para a bola passar sobre quem está na rede.',[[1.23,{rw:[.35,.3,-.4],rk:[.1,-.8,-.7]}],[1.53,{rk:[.1,.45,.9]}],[1.58,{rw:[.3,.65,.6],rk:[-.1,.85,.5]}],[1.88,{sy:-45,rw:[-.2,.9,.2],rk:[-.4,.9,0]}]],[[0,['RA','LA'],'Disfarce','Comece como um forehand de ataque: a mesma preparação esconde o lob.'],[1.23,['RKH','LKN','RKN'],'Raquete baixa','A raquete cai bem abaixo da bola e os joelhos flexionam: o caminho será íngreme, de baixo para cima.'],[1.53,['RW','RKH'],'Contato','Contato com a face um pouco aberta. O lob de topspin precisa de altura e rotação para cair atrás do adversário.'],[1.88,['SL','RW'],'Finalização alta','Termine bem alto, acompanhando a bola para cima.']]);
S.dp=cl(S.sl,'Drop shot','Drop shot: preparação igual à do slice ou do drive para disfarçar, raquete por baixo da bola com contato macio, backspin e a bola morrendo curta.',[[1.58,{rk:[-.05,.45,.9],rw:[-.25,.22,.55]}],[1.66,{rw:[-.2,.15,.6],rk:[.02,.3,.95]}],[2.03,{rw:[-.15,.1,.6],rk:[0,.2,.9],p:[0,0,.2]}]],[[0,['RA','LA'],'Split step','Pouse no instante em que o rival bate e saia já em direção à bola.'],[.42,['LKN','RKN'],'Arrancada','Primeiro passo na direção da bola, com o quadril e os ombros girando juntos.'],[.83,['RW','RKH'],'Disfarce','Preparação igual à do slice ou do drive: o rival só percebe tarde.'],[1.58,['RW','RKH'],'Raquete por baixo da bola','Face aberta e raquete por baixo da bola geram backspin e a bola morre curta. Contato macio, pouco swing.'],[2.03,['RW','LA'],'Finalização curta','Acompanhe a bola sem varrer e avance para a rede.']]);
// ===== Variações dos golpes =====
// Saque chapado = saque base (renomeado); slice, kick, corrida, swing volley, short angle e tweener como golpes próprios
S.sv.n='Saque chapado';
S.sv.intro='Saque chapado: lançamento à frente e na direção do alvo, contato no ponto mais alto com a face quase reta atrás da bola. É o mais rápido, com pouco efeito e menor margem sobre a rede. Costuma ser o primeiro saque.';
S.ss=cl(S.sv,'Saque slice','Saque slice: lançamento um pouco mais à direita (para destros) e contato na lateral externa da bola, varrendo da esquerda para a direita. O efeito lateral curva a bola e a faz escapar do devolvedor, com bom controle e menos velocidade que o chapado.',
 [[1.22,{lw:[-.02,1.15,.5],rk:[.05,.75,-.75]}],[1.52,{rk:[.1,.2,-.8],rw:[.34,.42,-.22]}],[1.62,{sy:36,hy:34,rk:[.15,.55,-.55]}],[1.7,{sy:16,hy:20,rw:[.27,1.15,.2],rk:[.32,.95,.22]}],[1.78,{sy:22,hy:18,rw:[.05,.62,.58],rk:[.18,-.5,.78]}],[2.1,{sy:-70,hy:-45}]],
 [[0,['HR','HL'],'Postura','Pegada continental e base estável. O slice sai do mesmo setup do saque chapado, o que dificulta a leitura do devolvedor.'],[.85,['LW','SL'],'Lançamento à direita','A bola sobe um pouco mais à direita e à frente (para destros) do que no saque chapado. Esse ponto permite acertar a lateral externa da bola.'],[1.22,['SR','LKN','RKN'],'Troféu','Joelhos flexionados, braço livre apontando para a bola e cotovelo alto. A preparação é a mesma do saque chapado.'],[1.52,['RKH','RE'],'Queda da raquete','A raquete cai atrás e sobe com o cotovelo à frente, preparando a rotação do antebraço.'],[1.7,['RW','RKH'],'Contato lateral','A face encontra o lado externo da bola e a raquete a varre da esquerda para a direita. O efeito lateral é o que curva a trajetória.'],[2.1,['SL','RA'],'Finalização','A raquete termina em torno do corpo, e a perna de trás avança para entrar na quadra e recuperar o equilíbrio.']]);
S.rf=cl(S.fh,'Forehand em corrida','Forehand em corrida: passos largos até a bola, base mais aberta, contato com o corpo ainda em movimento e finalização curta. O objetivo é controlar a bola (cruzada funda) e voltar rápido ao centro.',
 [[.78,{b:.12,l:8,rw:[.42,.2,-.3],ra:[.36,.05,-.3],la:[-.3,.05,.15]}],[1.23,{b:.18,l:18,rw:[.38,-.12,-.35],ra:[.34,.05,-.28],la:[-.3,.05,.25]}],[1.46,{l:22,ra:[.34,.05,-.2],la:[-.28,.05,.35]}],[1.53,{b:.1,l:22,p:[.05,0,.28],ra:[.36,.05,-.2],la:[-.3,.05,.45],rk:[.1,.12,1]}],[1.58,{b:.06,p:[.05,0,.32]}],[1.88,{b:.06,l:12,ra:[.3,.05,.2],la:[-.25,.05,.5]}]],
 [[0,['HR','HL'],'Leitura','Observe a direção da bola e saia cedo. Quanto antes o primeiro passo, mais equilibrado o contato.'],[.78,['SR','SL','HR'],'Preparação em movimento','Ombros giram durante a corrida: a raquete fica atrás sem esperar a parada completa.'],[1.23,['RKH','RKN','LKN'],'Passos de ajuste','Pequenos passos finais para acertar a distância. A perna de fora freia e a de dentro prepara o apoio.'],[1.53,['RW','RKH'],'Contato em equilíbrio','Contato à frente, base aberta e corpo inclinado para a bola. Bola cruzada e funda é a opção segura.'],[1.88,['SL','RA'],'Finalização curta','Termine sem exagerar o giro: a energia fica para a recuperação.'],[2.28,['RA','LA'],'Recuperação','Volte ao centro com passos cruzados e prepare a próxima bola.']]);
S.sw=cl(S.vl,'Swing volley','Swing volley: voleio com swing completo, em bola alta e sem quicar. Preparação com giro e raquete alta, swing de baixo para cima com topspin e contato à frente, atacando em vez de bloquear.',
 [[.35,{sy:75,hy:55,b:.18,l:16,rw:[.4,.45,-.3],lw:[-.3,.5,.3],rk:[.1,.9,-.5]}],[.55,{sy:68,hy:48,b:.22,l:20,p:[0,0,.12],rw:[.38,.2,-.2],lw:[-.35,.3,.2],rk:[.1,.15,-.85]}],[.65,{sy:6,hy:2,b:.2,l:20,p:[0,0,.3],rw:[.38,.25,.55],rk:[.1,.18,1]}],[.72,{sy:-30,hy:-18,rw:[.05,.55,.45],rk:[-.35,.55,.6]}],[.95,{sy:-34,hy:-20,rw:[-.1,.5,.4],rk:[-.4,.35,.55]}]],
 [[0,['HR','HL'],'Leitura da bola alta','Percebeu que a bola vai sobrar? Avance para atacar e não deixe quicar.'],[.35,['SR','SL','RKH'],'Giro e raquete alta','Ombros giram e a raquete sobe atrás da cabeça: essa preparação maior dá espaço para o swing.'],[.55,['RKH','RKN'],'Queda curta','A raquete cai abaixo da bola, menos que num forehand de fundo, para manter o controle perto da rede.'],[.65,['RW','RKH'],'Contato à frente','Bata à frente do corpo, com swing de baixo para cima e topspin. Atacar o espaço aberto, sem forçar.'],[.95,['SL','RW'],'Finalização','Termine sobre o ombro oposto e recupere o equilíbrio na rede para o próximo voleio.']]);
S.sa=cl(S.fh,'Short angle','Short angle: forehand que corta a quadra em ângulo fechado, com muito topspin, e cai perto da linha lateral e da rede. Exige contato à frente, pulso solto e uma finalização que acompanha o ângulo.',
 [[1.23,{rw:[.4,-.12,-.36],rk:[.1,-.45,-.88]}],[1.46,{hy:6,sy:36,rk:[.2,-.05,-.95]}],[1.53,{hy:2,sy:28,b:.16,l:18,p:[.06,0,.26],rw:[.42,.05,.45],rk:[.35,.1,.9]}],[1.58,{hy:-6,sy:2,rw:[.3,.35,.55],rk:[.05,.65,.6]}],[1.88,{hy:-26,sy:-58,rw:[-.3,.5,.1],rk:[-.65,-.3,-.45]}]],
 [[0,['HR','HL'],'Leitura','Bola curta: avance e procure o ângulo. O jogador que se aproxima ganha tempo e ângulo.'],[.78,['SR','SL','HR'],'Preparação com giro','Giro de ombros e quadril como no forehand padrão: o disfarce depende de manter a mesma preparação.'],[1.23,['RKH','RKN'],'Raquete por baixo da bola','A raquete cai baixa para subir rapidamente, com muito topspin: a bola precisa cair rápido depois de passar a rede.'],[1.53,['RW','RKH'],'Contato à frente e lateral','Contato mais à frente e levemente de fora para dentro. A face guia a bola para o canto e a rotação a faz cair.'],[1.88,['SL','RW'],'Finalização no ângulo','A raquete acompanha o ângulo, passando pelo outro lado do corpo.'],[2.28,['RA','LA'],'Recuperação','Volte ao centro: o short angle abre a quadra, mas deixa espaço atrás.']]);
S.tw={n:'Tweener',dur:2.6,tc:1.2,inb:[.1,.9,2.6],intro:'Tweener: golpe de exceção, dado entre as pernas e de costas para a rede, quando a bola passou e não dá tempo de virar. A raquete passa de baixo para trás, com o corpo agachado. É mais um recurso de emergência (ou de exibição) do que uma arma tática.',
 keys:[K(0,{}),K(.35,{hy:70,sy:90,b:.14,l:6,p:[0,0,-.3],rw:[.35,.2,-.2],lw:[-.3,.3,.2],rk:[.1,.6,-.5],ra:[.25,.05,-.35],la:[-.2,.05,.05]}),K(.8,{hy:140,sy:158,b:.22,l:-18,p:[0,0,-.7],rw:[-.1,.2,-.3],lw:[.45,.35,-.1],rk:[0,.2,-.9],ra:[-.35,.05,-.55],la:[.3,.05,-.75]}),K(1,{hy:176,sy:182,b:.32,l:-56,p:[0,-.02,-.95],rw:[.04,-.1,-.12],lw:[.55,.2,-.2],rk:[0,-.95,.15],ra:[-.38,.05,-.8],la:[.38,.05,-.9]}),K(1.2,{hy:180,sy:186,b:.32,l:-56,p:[0,-.02,-.95],rw:[.04,-.08,-.05],lw:[.55,.25,-.25],rk:[0,-.12,1],ra:[-.38,.05,-.8],la:[.38,.05,-.9]}),K(1.35,{hy:176,sy:176,b:.3,l:-50,p:[0,0,-.9],rw:[.05,0,.15],lw:[.5,.3,-.2],rk:[0,.5,.85],ra:[-.38,.05,-.8],la:[.38,.05,-.9]}),K(1.8,{hy:100,sy:110,b:.14,l:0,p:[0,0,-.6],rw:[.3,.2,.1],lw:[-.2,.3,.2],rk:[.1,.5,.6],ra:[-.1,.05,-.5],la:[.2,.05,-.4]}),K(2.4,{p:[0,0,-.1]})],
 tips:[[0,['HR','HL'],'Leitura','A bola passou: não dá tempo de virar para um golpe normal. Decida cedo e saia.'],[.35,['SR','SL','HR'],'Giro e corrida','Quadril e ombros giram enquanto o jogador corre de costas para a rede, acompanhando a bola por cima do ombro.'],[.8,['LKN','RKN'],'De costas para a rede','Base larga, joelhos flexionados e tronco inclinado: o espaço entre as pernas precisa ser livre para a raquete passar.'],[1,['RW','RKH'],'Raquete por baixo','A raquete desce entre as pernas. O braço esquerdo se abre para equilibrar o corpo.'],[1.2,['RW','RKH'],'Contato entre as pernas','Contato atrás do quadril, com a face dirigida para a quadra adversária. É um golpe curto: a bola sai com o ritmo que chegou, sem swing completo.'],[1.8,['SL','RA'],'Recuperação','Gire de volta para a quadra: o golpe deixa o jogador mal posicionado.']]};
// ORDEM_GOLPES: agrupa as variações ao lado dos golpes-base nos chips
for(const k of ['fh','rf','sa','bh','sv','ss','kk','b1','sl','sm','vl','sw','ap','rt','lb','dp','tw']){const v=S[k];delete S[k];S[k]=v}
for(const id in S){const s=S[id];s.tips=s.tips.sort((a,b)=>a[0]-b[0]).map(a=>({t:a[0],j:a[1],h:a[2],x:a[3]}))}

  // ===== Tabela Balística e Tática dos Golpes (SH) =====
const SH=[
['fh','Cruzado profundo',2.2,-1.2,-3.3,22.3,32,2700,18,11,'Padrão de rali: o cruzado passa pela parte mais baixa da rede e usa a diagonal, que é a mais longa. Federer: ~2.700 rpm em média.'],
['fh','Paralela',2.2,-1.2,3.6,22,34,2700,16,9,'Mudança de direção: mais risco (rede mais alta, quadra mais curta). Funciona quando o rival já correu para o cruzado.'],
['fh','Inside-out',-1.8,-1.2,3.6,22,35,3000,18,11,'Contorna o backhand e bate o forehand na diagonal para o backhand do rival. Padrão clássico de Federer.'],
['fh','Topspin pesado',2.2,-1.8,-3.3,22.8,30,3200,18,15,'Face ~15° fechada e caminho ~+18° (medido em Nadal). Média ~3.200 rpm; a bola quica alto e empurra o rival para trás.'],
['bh','Cruzado 2 mãos',-2.2,-1.2,3.3,22.3,30,2400,16,10,'A mão esquerda comanda o golpe. O cruzado profundo é o padrão de rali do backhand.'],
['bh','Paralela 2 mãos',-2.2,-1.2,-3.6,22,31,2400,14,9,'Troca de direção atrás de quem já se moveu para o cruzado.'],
['b1','Cruzado 1 mão',-2.2,-1.2,3.3,22.3,31,2000,14,9,'Estilo Federer: contato mais à frente com o braço estendido e o outro braço para trás como contrapeso.'],
['b1','Paralela 1 mão',-2.2,-1.2,-3.6,22,32,2000,12,8,'Arma de troca de direção. Mantenha o contato à frente do corpo.'],
['sl','Slice curto cruzado',-1.8,-1,3.2,15.5,20,-2200,-12,-16,'Bola baixa que obriga o rival a subir até a rede. Combinação clássica de Federer para abrir a quadra.'],
['sl','Slice fundo paralela',-1.8,-1,-3.6,21.5,24,-2200,-10,-14,'Mantém a bola baixa e funda; prepara a aproximação ou a troca de ritmo.'],
['sv','T no lado par',1,-.6,-.3,17.7,52,1800,10,5,'Federer: ~49% no T no 1º saque do lado par (amostra de dados charting). A rota do braço muda pouco: o timing da rotação define a direção.'],
['sv','Aberto no lado par',1,-.6,-3.9,17.7,50,2200,10,5,'~45% dos primeiros saques de Federer no lado par (amostra). Abre a quadra para o forehand seguinte.'],
['sv','T no lado ímpar',-1,-.6,.3,17.7,52,1800,10,5,'~39% dos primeiros saques de Federer no lado ímpar foram no T (amostra).'],
['sv','Aberto no lado ímpar',-1,-.6,3.9,17.7,50,2400,10,5,'~57% dos primeiros saques de Federer no lado ímpar foram abertos (amostra), na direção do backhand do rival.'],
['sm','Smash cruzado',.5,6.5,-3,20.5,36,600,-20,16,'Contato alto, braço estendido e finalização cruzando o corpo. Mire o lado aberto da quadra.'],
['vl','Voleio cruzado',1,9.3,-3,18.5,22,-900,-5,-8,'Bloqueio curto com backspin, direção ao espaço aberto.'],
['vl','Voleio paralelo',1,9.3,3,18.5,22,-900,-5,-8,'Tire o rival do centro. Mantenha a raquete à frente.'],
['sv','Slice no lado par',1,-.6,-3.9,17.7,47,2200,8,5,'Mais spin (~2.200 rpm) e menos velocidade (~170 km/h) que o saque plano: a bola curva e quica baixo.'],
['kk','Kick aberto no lado ímpar',-1,-.6,3.8,17.5,41.7,3200,30,0,'Kick: ~150 km/h e ~3.200 rpm. A bola cai mais no fim e quica alto, empurrando o rival para trás.'],
['kk','Kick no corpo lado par',1,-.6,-1.2,17.5,41.7,3200,30,0,'Kick no corpo: o quique alto tira o tempo de reação e a bola sobe na altura do ombro.'],
['rt','Cruzada profunda',1.8,-2.4,-3,22,30,2400,16,10,'Devolução cruzada funda: usa a diagonal e tira o ângulo do sacador na terceira bola.'],
['rt','Centro (segura)',1.8,-2.4,0,21.5,28,2000,14,9,'Devolução funda pelo centro: pouco ângulo para o sacador. Opção segura contra saques fortes.'],
['rt','Chip aos pés',1.8,-2.2,-1.5,19.5,22,-1800,-8,-12,'Chip: bloqueio curto com backspin e baixo, aos pés de quem sobe à rede.'],
['lb','Lob de topspin',2,-1.5,-1.5,21.5,22,2200,40,10,'Lob de ataque: a bola sobe rápido e cai antes da linha, com topspin, atrás de quem está na rede.'],
['lb','Lob defensivo',2,-1.5,1.5,21.5,20,-800,30,-25,'Lob defensivo: face aberta, alto e profundo para ganhar tempo e reposicionar.'],
['dp','Drop cruzado',-1,4,3,14.8,13,-2200,-15,-22,'Drop shot: backspin forte e contato macio fazem a bola morrer logo depois da rede. Disfarce é essencial.'],
['ap','Aproximação paralela',1.5,3,3.6,21.5,28,2000,16,9,'Bola funda na paralela encurta o caminho até a rede e cobre a passada. Cerca de 96% das aproximações de elite caem dentro (dados charting).']];
SH.push(
['ss','Slice aberto no lado par',1,-.6,-3.9,17.5,47,2200,8,5,'Slice aberto: o efeito lateral curva a bola para fora e o quique baixo afasta o devolvedor da quadra. Mais controle e menos velocidade que o chapado (~170 km/h).'],
['ss','Slice no corpo lado par',1,-.6,-1.2,17.5,46,2000,8,5,'Slice no corpo: o devolvedor perde espaço para o swing e o quique baixo atrapalha a devolução.'],
['ss','Slice aberto no lado ímpar',-1,-.6,3.9,17.5,47,2200,8,5,'No lado ímpar, o slice aberto leva a bola para o backhand do devolvedor.'],
['rf','Corrida cruzada funda',3.3,-1.5,-3.3,22,27,2600,14,10,'Forehand em corrida, cruzado e fundo: usa a diagonal, dá tempo para recuperar e é a opção segura quando se está deslocado.'],
['rf','Corrida paralela (contra-ataque)',3.3,-1.5,3.6,21.5,28,2500,14,9,'Contra-ataque na paralela: surpreende quem já se moveu para o cruzado. Mais risco, pois a quadra é mais curta e a rede mais alta.'],
['rf','Corrida defensiva alta',3.6,-1.8,-2.5,22.5,22,2400,20,12,'Bola alta e funda para ganhar tempo e voltar ao centro. É a resposta quando a corrida deixa o jogador sem equilíbrio.'],
['sw','Swing volley cruzado',1,8.8,-3.2,19,30,1800,8,14,'Swing volley cruzado: ataque em bola alta, sem quicar, com topspin para a bola cair na quadra.'],
['sw','Swing volley paralelo',1,8.8,3.4,19,30,1800,8,14,'Swing volley na paralela: tira o rival do centro. Mantenha o contato à frente e a finalização controlada.'],
['sa','Short angle cruzado',2,1.8,-3.9,16,24,3200,14,20,'Short angle cruzado: topspin forte faz a bola cair rápido depois da rede, perto da linha lateral e da linha de serviço, abrindo a quadra.'],
['sa','Short angle de contra-ataque',2.6,.8,-3.9,18,26,3000,14,18,'Short angle em contra-ataque: aproveita uma bola curta do rival para atacar o espaço aberto com ângulo e rotação.'],
['tw','Tweener cruzado',-.3,.5,-2.5,18,21,1500,0,0,'Tweener: o golpe sai com pouca velocidade e precisa de uma bola descendo à frente do corpo. Serve como recurso de emergência ou exibição.'],
['tw','Tweener paralelo',-.3,.5,2.5,18,21,1500,0,0,'Tweener na paralela: a direção depende da face no contato, que é difícil de controlar. Evite usar em pontos decisivos.']
);
for(let i=SH.length-1;i>=0;i--)if(SH[i][0]=='sv'&&SH[i][1]=='Slice no lado par')SH.splice(i,1);

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

  function flightPosFromSim(flight, u) {
    if (!flight || !flight.P || u > flight.T) return null;
    const P = flight.P;
    for (let i = 1; i < P.length; i++) {
      if (P[i][0] >= u) {
        const a = P[i - 1], b = P[i];
        const k = (u - a[0]) / (b[0] - a[0] || 1);
        return [0, 1, 2].map((j) => a[j + 1] + (b[j + 1] - a[j + 1]) * k);
      }
    }
    return null;
  }

  function analyzePoseComparison(stroke, t, observed = {}, opts = {}) {
    const p = pose(stroke, t, opts);
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
    rad, deg, clampBio, V, rotAroundAxis, ik, BASE, GRIPS,
    joints, evalPose, poseRaw, pose, motion,
    sim, solve, flightPosFromSim, analyzePoseComparison,
    S, SH,
  };
})(typeof window !== 'undefined' ? window : globalThis);
