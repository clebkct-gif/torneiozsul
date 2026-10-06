const CACHE = 'x1-zsul-v18';
const FONTS_CSS = 'https://fonts.googleapis.com/css2?family=Montserrat:wght@500;700;800;900&family=Orbitron:wght@800;900&family=Teko:wght@700&display=swap';
const LOCAL = ['./', 'manifest.webmanifest', 'icon.svg', 'icon-192.png', 'icon-512.png'];

self.addEventListener('install', e => {
  e.waitUntil((async () => {
    const c = await caches.open(CACHE);
    await Promise.allSettled(LOCAL.map(u => c.add(u)));
    try {
      const css = await fetch(FONTS_CSS);
      const text = await css.clone().text();
      await c.put(FONTS_CSS, css);
      const urls = [...text.matchAll(/url\((https:[^)]+)\)/g)].map(m => m[1]);
      await Promise.allSettled(urls.map(u => c.add(u)));
    } catch (_) {}
    self.skipWaiting();
  })());
});

self.addEventListener('activate', e => {
  e.waitUntil((async () => {
    for (const k of await caches.keys()) if (k !== CACHE) await caches.delete(k);
    await self.clients.claim();
  })());
});

self.addEventListener('fetch', e => {
  const req = e.request;
  if (req.method !== 'GET') return;
  if (req.mode === 'navigate') {            // página: rede primeiro, cache se offline
    e.respondWith((async () => {
      try {
        const res = await fetch(req);
        (await caches.open(CACHE)).put(req, res.clone());
        return res;
      } catch (_) {
        return (await caches.match(req)) || (await caches.match('./')) || Response.error();
      }
    })());
    return;
  }
  e.respondWith((async () => {              // demais: cache primeiro
    const hit = await caches.match(req);
    if (hit) return hit;
    try {
      const res = await fetch(req);
      if (res.ok || res.type === 'opaque') (await caches.open(CACHE)).put(req, res.clone());
      return res;
    } catch (_) { return Response.error(); }
  })());
});
