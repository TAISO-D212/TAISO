import firebase from 'firebase/app';
import 'firebase/messaging';
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

firebase.initializeApp(firebaseConfig);

const messaging = firebase.messaging();

export function requestPermission() {
	void Notification.requestPermission().then((permission) => {
		if (permission === 'granted') {
			messaging
				.getToken({ vapidKey: process.env.REACT_APP_FIREBASE_VAPID_KEY })
				.then((token: string) => {
					console.log(`푸시 토큰 발급 완료 : ${token}`);
				})
				.catch((err) => {
					console.log('푸시 토큰 가져오는 중에 에러 발생');
				});
		} else if (permission === 'denied') {
			console.log('푸시 권한 차단');
		}
	});
}
