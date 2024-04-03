importScripts("https://www.gstatic.com/firebasejs/8.10.1/firebase-app.js");
importScripts(
  "https://www.gstatic.com/firebasejs/8.10.1/firebase-messaging.js"
);

const firebaseConfig = {
  apiKey: "AIzaSyBm16pVX-h-wlTc0Ll7I9otOl_l5_CiXDM",
  projectId: "taiso-18ea8",
  messagingSenderId: "37210886537",
  appId: "1:37210886537:web:f34fb3edec5872323c9158",
};

const firebaseApp = firebase.initializeApp(firebaseConfig);

// Initialize Firebase Cloud Messaging and get a reference to the service
const messaging = firebase.messaging();

messaging.onBackgroundMessage((payload) => {
  console.log(
    "[firebase-messaging-sw.js] Received background message ",
    payload
  );
  // TODO : payload를 이용한 notification 생성
  const notificationTitle = payload.notification.title;
  const notificationOptions = {
    body: payload.notification.body,
    icon: "/firebase-logo.png",
  };

  self.registration.showNotification(notificationTitle, notificationOptions);
});

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
