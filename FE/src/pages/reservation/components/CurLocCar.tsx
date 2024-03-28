import { useEffect, useRef, useState } from 'react';
import CurLocMarker from '../../../assets/icon/CurLoc_Img.png';
import LatLngAddStore from '../../../store/LatLngAddStore';
import LoadingBus from '../../../assets/loading/loading.gif';

declare global {
	interface Window {
		kakao: any;
	}
}

export const CurLocCar = () => {
	const { currentLat, currentLng } = LatLngAddStore((state) => state);
	const latitude = currentLat;
	const longitude = currentLng;
	const mapRef = useRef<any>(null);
	const [locationList, setLocationList] = useState<kakao.maps.LatLng[]>([]);
	const [map, setMap] = useState<any>();
	const [curMarker, setCurMarker] = useState<any>();
	const polylineRef = useRef<kakao.maps.Polyline | null>(null);
	const [location, setLocation] = useState({
		center: {
			lat: latitude,
			lng: longitude,
		},
		isLoading: true,
	});

	useEffect(() => {
		// const container = document.getElementById('map');
		const container = mapRef.current;
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
			offset: new window.kakao.maps.Point(0, 0),
		};

		const markerImage = new window.kakao.maps.MarkerImage(imageSrc, imageSize, imgOptions);
		const newCurMarker = new window.kakao.maps.Marker({
			zIndex: 1,
			image: markerImage,
		});

		setMap(newMap);
		setCurMarker(newCurMarker);
	}, []);

	useEffect(() => {
		if (map && 'geolocation' in navigator) {
			navigator.geolocation.watchPosition(
				(position) => {
					const lat = position.coords.latitude;
					const lng = position.coords.longitude;
					setLocation((pre) => ({
						...pre,
						center: { lat, lng },
						isLoading: false,
					}));
					setLocationList((pre) => [...pre, new kakao.maps.LatLng(lat, lng)]);
				},
				(error) => {
					console.log(error);
				},
				{
					enableHighAccuracy: true,
					maximumAge: 1000,
					timeout: 2000,
				},
			);
		} else {
			console.log('Geolocation is not available.');
		}

		// 클린업 함수
		// return () => {
		//   if (watchId !== null) {
		//     navigator.geolocation.clearWatch(watchId);
		//   }
		// };
	}, []);

	useEffect(() => {
		curMarker?.setMap(null);
		if (map && locationList.length > 0 && window.kakao.maps) {
			if (!polylineRef.current) {
				const polyline = new window.kakao.maps.Polyline({
					path: locationList,
					strokeWeight: 5,
					strokeColor: '#FFAE00',
					strokeOpacity: 0.7,
					strokeStyle: 'solid',
				});
				polyline.setMap(map);
				polylineRef.current = polyline;
			} else {
				polylineRef.current.setPath(locationList);
			}
			map?.panTo(locationList[-1]);
			curMarker?.setPosition(locationList[-1]);
			curMarker?.setMap(map);
		}
	}, [locationList, location.center]);

	return (
		<>
			{location?.isLoading === false ? (
				<div ref={mapRef} className='fixed top-0 w-[100%] h-[100%] animate-fadeIn'></div>
			) : (
				<div className='flex justify-center items-center'>
					<img src={LoadingBus} alt='LOADING' />
				</div>
			)}
		</>
	);
};
