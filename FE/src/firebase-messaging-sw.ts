import { initializeApp } from 'firebase/app';
import { getToken, onMessage } from 'firebase/messaging';
import { getMessaging, onBackgroundMessage } from 'firebase/messaging/sw';
import { viteConfig } from './apis/viteConfig';

const firebaseConfig = {
	apiKey: viteConfig.VITE_FIREBASE_API_KEY,
	authDomain: viteConfig.VITE_FIREBASE_AUTH_DOMAIN,
	projectId: viteConfig.VITE_FIREBASE_PROJECT_ID,
	storageBucket: viteConfig.VITE_FIREBASE_STORAGE_BUCKET,
	messagingSenderId: viteConfig.VITE_FIREBASE_MESSAGING_SENDER_ID,
	appId: viteConfig.VITE_FIREBASE_APP_ID,
	measurementId: viteConfig.VITE_FIREBASE_MEASUREMENT_ID,
};

const app = initializeApp(firebaseConfig);

// Initialize Firebase Cloud Messaging and get a reference to the service
const messaging = getMessaging(app);

const fetchFCMToken = async (token: any) => {
	const response = await fetch(`http://localhost:3000/events`, {
		method: 'POST',
		body: JSON.stringify(token),
		headers: {
			'Content-Type': 'application/json',
		},
	});

	if (!response.ok) {
		const error = new Error('An error occurred while fetching FCMtoken.');
		throw error;
	} else if (response.ok) {
		console.log('FCMtoken fetched successfully.');
	}

	const res = await response.json();

	return res.data;
};

const requestPermission = async () => {
	console.log('Requesting permission...');
	const permission = await Notification.requestPermission();

	if (permission === 'granted') {
		console.log('Notification permission granted.');
	} else if (permission === 'denied') {
		console.log('Notification permission denied.');
		return;
	}

	const token = await getToken(messaging, { vapidKey: viteConfig.VITE_FIREBASE_PUBLIC_VAPID_KEY });

	if (token) {
		console.log(`푸시 토큰 발급 완료 : ${token}`);
		// Send the token to your server and update the UI if necessary
		// if (
		// 	localStorage.getItem('FCMtoken') === null ||
		// 	localStorage.getItem('FCMtoken') !== currentToken
		// ) {
		// 	// TODO : 백엔드로 토큰을 보내줘야 백엔드로부터 푸시 알림이 오는가? 테스트 할 때는 내가 받은 토큰을 저장해야만 가능!
		// 	const fetchRes = fetchFCMToken(currentToken);
		// 	console.log(fetchRes);
		// 	localStorage.setItem('FCMtoken', currentToken);
		// }
	} else {
		// Show permission request UI
		console.log('No registration token available. Request permission to generate one.');
		// ...
	}
	onMessage(messaging, (payload) => {
		console.log('Message received. : ', payload);
		// ...
		let notificationPermission = Notification.permission;

		if (notificationPermission === 'granted') {
			//Notification을 이미 허용한 사람들에게 보여주는 알람창
			new Notification(payload.notification.title, {
				body: payload.notification.body,
				icon: '/assets/icon/icon_48.png',
			});
		} else if (notificationPermission !== 'denied') {
			//Notification을 거부했을 경우 재 허용 창 띄우기
			Notification.requestPermission(function (permission) {
				if (permission === 'granted') {
					new Notification(payload.notification.title, {
						body: payload.notification.body,
					});
				} else {
					alert('알람 허용이 거부되었습니다.');
				}
			});
		}
	});
	// onBackgroundMessage(messaging, (payload) => {
	// 	console.log('[firebase-messaging-sw.js] Received background message ', payload);
	// 	// TODO : payload를 이용한 notification 생성
	// 	const notificationTitle = 'Background Message Title';
	// 	const notificationOptions = {
	// 		body: 'Background Message body.',
	// 		icon: '/firebase-logo.png',
	// 	};
	// });
};

requestPermission();
