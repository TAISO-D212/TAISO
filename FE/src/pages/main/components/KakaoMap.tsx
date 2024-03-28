/* eslint-disable @typescript-eslint/no-explicit-any */
import { useState, useEffect } from 'react';
import CurrentLoc from '../../../assets/image/CurrentLoc.png';
import CurLocMarker from '../../../assets/icon/CurLoc_Img.png';
import LatLngAddStore from '../../../store/LatLngAddStore';

declare global {
	interface Window {
		kakao: any;
	}
}

export const KakaoMap = () => {
	const [map, setMap] = useState<any>();
	const [curMarker, setCurMarker] = useState<any>();

	const { currentLat, currentLng } = LatLngAddStore((state) => state);
	const latitude = currentLat;
	const longitude = currentLng;

	// 1) 카카오맵 불러오기
	useEffect(() => {
		window.kakao.maps.load(() => {
			const container = document.getElementById('map');
			const curLoc = new window.kakao.maps.LatLng(latitude, longitude);
			const options = {
				center: curLoc,
				level: 4,
			};

			const newMap = new window.kakao.maps.Map(container, options);
			const mapTypeControl = new window.kakao.maps.MapTypeControl();
			newMap.addControl(mapTypeControl, window.kakao.maps.ControlPosition.TOPRIGHT);

			const imageSrc = CurLocMarker;
			const imageSize = new window.kakao.maps.Size(40, 40);

			const imgOptions = {
				offset: new window.kakao.maps.Point(20, 0),
			};

			const markerImage = new window.kakao.maps.MarkerImage(imageSrc, imageSize, imgOptions);
			const newCurMarker = new window.kakao.maps.Marker({
				image: markerImage,
			});

			setMap(newMap);
			setCurMarker(newCurMarker);
		});
	}, []);

	// 2) 현재 위치 함수
	const goCurrentLoc = () => {
		navigator.geolocation.getCurrentPosition(
			getPosSuccess,
			() => alert('위치 정보를 가져오는데 실패했습니다.'),
			{
				enableHighAccuracy: true,
				maximumAge: 30000,
				timeout: 27000,
			},
		);
	};

	// 3) 정상적으로 현재위치 가져올 경우 실행
	const getPosSuccess = (pos: GeolocationPosition) => {
		// 현재 위치(위도, 경도) 가져온다.
		const currentPos = new window.kakao.maps.LatLng(
			pos.coords.latitude, // 위도
			pos.coords.longitude, // 경도
		);

		// // 인포윈도우를 생성합니다
		// const curLocInfowindow = new kakao.maps.InfoWindow({
		// 	content: '<div style="padding:5px;">내 위치</div>',
		// 	removable: true,
		// });

		// // 인포윈도우를 마커위에 표시합니다
		// curLocInfowindow.open(map, curMarker);

		// 지도를 이동 시킨다.
		map.panTo(currentPos);

		// 기존 마커를 제거하고 새로운 마커를 넣는다.
		curMarker.setMap(null);
		curMarker.setPosition(currentPos);
		curMarker.setMap(map);
	};

	return (
		<>
			<div id='map' className='fixed top-0 w-[100%] h-[70%] animate-fadeIn'></div>
			<div
				className='fixed w-[50px] h-[50px] bottom-[45%] right-[9px] z-10 flex flex-col justify-center items-center'
				onClick={goCurrentLoc}>
				<img src={CurrentLoc} alt='HOME_TAB' />
			</div>
		</>
	);
};
