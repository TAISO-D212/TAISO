import { useState, useEffect } from 'react';
import CurrentLoc from '../../../assets/image/CurrentLoc.png';
import { watchPositionHook } from '../../../hooks/watchPositionHook';
import LatLngAddStore from '../../../store/LatLngAddStore';

// kakao 변수를 전역으로 선언
declare global {
	interface Window {
		kakao: any;
	}
}

interface IMarker {
	setMap: (map: any) => void;
	setPosition: (position: any) => void;
}

interface IInfowindow {
	setContent: (content: any) => void;
	open: (map: any, marker: any) => void;
}

export const KakaoMap = () => {
	const [map, setMap] = useState<any>();
	const [userMarker, setUserMarker] = useState<IMarker | undefined>();
	const [ps, setPs] = useState<any>();
	const [infowindow, setInfowindow] = useState<IInfowindow | undefined>();
	const [keyword, setKeyword] = useState<string>('');

	const { currentLat, currentLng } = LatLngAddStore((state) => state);
	const lat = currentLat;
	const long = currentLng;

	let markers: any[] = [];

	useEffect(() => {
		const mapContainer = document.getElementById('map') as HTMLElement;
		const mapOption = {
			center: new window.kakao.maps.LatLng(lat, long),
			level: 3,
		};

		const newMap = new window.kakao.maps.Map(mapContainer, mapOption);

		const mapTypeControl = new window.kakao.maps.MapTypeControl();

		newMap.addControl(mapTypeControl, window.kakao.maps.ControlPosition.TOPRIGHT);

		const newPs = new window.kakao.maps.services.Places();

		const newInfowindow = new window.kakao.maps.InfoWindow({
			zIndex: 1,
			removable: true,
		});

		setMap(newMap);
		setUserMarker(new window.kakao.maps.Marker());
		setPs(newPs);
		setInfowindow(newInfowindow);
	}, []);

	const handleKeywordChange = (e: React.ChangeEvent<HTMLInputElement>) => {
		setKeyword(e.target.value);
	};

	function searchPlaces(e: React.FormEvent<HTMLFormElement>) {
		e.preventDefault();
		const searchOption = {
			location: new window.kakao.maps.LatLng(lat, long),
			radius: 20000,
			sort: window.kakao.maps.services.SortBy.DISTANCE,
		};
		ps.keywordSearch(keyword, placesSearchCB, searchOption);
	}

	function placesSearchCB(data: any, status: any, pagination: any) {
		if (status === window.kakao.maps.services.Status.OK) {
			displayPlaces(data);
		}
	}

	function displayPlaces(places: any[]) {
		const newBound = new window.kakao.maps.LatLngBounds();

		removeMarker();

		for (let i = 0; i < places.length; i++) {
			const placePosition = new window.kakao.maps.LatLng(places[i].y, places[i].x);
			let marker = addMarker(placePosition, i);

			newBound.extend(placePosition);

			(function (marker, title) {
				window.kakao.maps.event.addListener(marker, 'mouseover', function () {
					displayInfowindow(marker, title);
				});

				window.kakao.maps.event.addListener(marker, 'mouseout', function () {
					infowindow.close();
				});
			})(marker, places[i].place_name);
		}

		map.setBounds(newBound);
	}

	function addMarker(position: any, idx: number) {
		const imageSrc =
			'https://t1.daumcdn.net/localimg/localimages/07/mapapidoc/marker_number_blue.png';
		let imageSize = new window.kakao.maps.Size(36, 37);

		let imgOptions = {
			spriteSize: new window.kakao.maps.Size(36, 691),
			spriteOrigin: new window.kakao.maps.Point(0, idx * 46 + 10),
			offset: new window.kakao.maps.Point(13, 37),
		};

		let markerImage = new window.kakao.maps.MarkerImage(imageSrc, imageSize, imgOptions);
		let marker = new window.kakao.maps.Marker({
			position: position,
			image: markerImage,
		});

		marker.setMap(map);
		markers.push(marker);

		return marker;
	}

	function removeMarker() {
		for (let i = 0; i < markers.length; i++) {
			markers[i].setMap(null);
		}
		markers = [];
		setUserMarker(markers);
	}

	function displayInfowindow(marker: any, title: string) {
		let content = '<div style="padding:5px;z-index:1;color:black;">' + title + '</div>';

		infowindow.setContent(content);
		infowindow.open(map, marker);
	}

	const goCurrentLoc = () => {
		navigator.geolocation.getCurrentPosition(
			getPosSuccess,
			() => alert('위치 정보를 가져오는데 실패했습니다.'),
			{
				enableHighAccuracy: true,
				maximumAge: 20000,
				timeout: 10000,
			},
		);
	};

	function displayMarker(locPosition: any, message: any) {
		// 마커를 생성합니다
		let curLocMarker = new kakao.maps.Marker({
			map: map,
			position: locPosition,
		});

		setUserMarker(curLocMarker);

		let iwContent = message; // 인포윈도우에 표시할 내용
		let iwRemoveable = true;

		// 인포윈도우를 생성합니다
		let curLocInfowindow = new kakao.maps.InfoWindow({
			content: iwContent,
			removable: iwRemoveable,
		});

		setInfowindow(undefined);

		// 인포윈도우를 마커위에 표시합니다
		curLocInfowindow.open(map, curLocMarker);

		// 지도 중심좌표를 접속위치로 변경합니다
		map.panTo(locPosition);
	}

	const getPosSuccess = (pos: GeolocationPosition) => {
		let currentPos = new window.kakao.maps.LatLng(pos.coords.latitude, pos.coords.longitude);
		let message = '<div style="padding:5px;">내 위치</div>';

		removeMarker();

		displayMarker(currentPos, message);
	};
	return (
		<>
			<div id='map' style={{ width: '100vw', height: '100vh' }} className='animate-fadeIn'></div>;
			<div className='fixed z-10 w-[65%] h-[35px] top-[5px] left-[5px] flex justify-evenly items-center border border-grey'>
				<form onSubmit={searchPlaces} className='px-[5px]'>
					키워드 :{' '}
					<input className='w-[45%]' type='text' value={keyword} onChange={handleKeywordChange} />
					<button type='submit' className='w-[20%]'>
						검색
					</button>
				</form>
			</div>
			<div
				className='fixed w-[50px] h-50px] bottom-[90px] right-[9px] z-10 flex flex-col justify-center items-center'
				onClick={goCurrentLoc}>
				<img src={CurrentLoc} alt='HOME_TAB' />
			</div>
		</>
	);
};
