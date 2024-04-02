import { initializeApp } from 'firebase/app';
import { getToken, onMessage } from 'firebase/messaging';
import { getAnalytics } from 'firebase/analytics';
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
const analytics = getAnalytics(app);

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

	const token = await getToken(messaging, { vapidKey: '<YOUR_PUBLIC_VAPID_KEY_HERE>' });

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
	});
	onBackgroundMessage(messaging, (payload) => {
		console.log('[firebase-messaging-sw.js] Received background message ', payload);
		// TODO : payload를 이용한 notification 생성
		const notificationTitle = 'Background Message Title';
		const notificationOptions = {
			body: 'Background Message body.',
			icon: '/firebase-logo.png',
		};
	});
};

requestPermission();
