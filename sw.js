const CACHE_NAME = 'torneio-v1';
const ASSETS = [
  './index.html',
  './manifest.json',
  './logo.jpg'
];

self.addEventListener('install', e => {
  e.waitUntil(caches.open(CACHE_NAME).then(cache => cache.addAll(ASSETS)));
});

self.addEventListener('fetch', e => {
  // Estratégia: Tenta ir na rede buscar o arquivo mais atual do GitHub, se falhar pega o cache
  e.respondWith(
    fetch(e.request)
      .then(res => {
        const copy = res.clone();
        caches.open(CACHE_NAME).then(cache => cache.put(e.request, copy));
        return res;
      })
      .catch(() => caches.match(e.request))
  );
});
