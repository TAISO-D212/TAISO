const CACHE_NAME = "TAISO-CACHE-V1";

// 캐싱할 파일
const FILES_TO_CACHE = ["../offline.html", "../assets/icon/icon_48.png"];

// 상술한 파일 캐싱
self.addEventListener("install", (event) => {
  event.waitUntil(
    caches
      .open(CACHE_NAME)
      .then((cache) => cache.addAll(FILES_TO_CACHE))
      .then(() => self.skipWaiting())
  );
});

// CACHE_NAME이 변경되면 오래된 캐시 삭제
self.addEventListener("activate", (event) => {
  event.waitUntil(
    caches.keys().then((keyList) =>
      Promise.all(
        keyList.map((key) => {
          if (CACHE_NAME !== key) return caches.delete(key);
        })
      )
    )
  );
});

// 요청에 실패하면 오프라인 페이지 표시
self.addEventListener("fetch", (event) => {
  if ("navigate" !== event.request.mode) return;

  event.respondWith(
    fetch(event.request).catch(() =>
      caches.open(CACHE_NAME).then((cache) => cache.match("../offline.html"))
    )
  );
});

// 웹 푸시 알림 수신
self.addEventListener("push", function (event) {
  console.log("push: ", event.data.json());
  if (!e.data.json()) return;

  const resultData = event.data.json().notification;
  const notificationTitle = resultData.title;
  const notificationOptions = {
    body: resultData.body,
    icon: resultData.image,
    tag: resultData.tag,
    ...resultData,
  };
  console.log("push: ", { resultData, notificationTitle, notificationOptions });

  self.registration.showNotification(notificationTitle, notificationOptions);
});

self.addEventListener("notificationclick", function (event) {
  console.log("notification click");
  // TODO : MovingTAISO 페이지로 이동
  const url = "/";
  event.notification.close();
  event.waitUntil(clients.openWindow(url));
});
