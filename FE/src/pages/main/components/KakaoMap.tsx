import { useState, useEffect } from 'react';
import CurrentLoc from '../../../assets/image/CurrentLoc.png';
import CurLocImage from '../../../assets/icon/CurLoc_Img.png';
import LatLngAddStore from '../../../store/LatLngAddStore';
import { watchPositionHook } from '../../../hooks/watchPositionHook';

// kakao 변수를 전역으로 선언
declare global {
	interface Window {
		kakao: any;
	}
}

interface IUserMarker {
	setMap: (map: any) => void;
	setPosition: (position: any) => void;
}

interface IInfowindow {
	setContent: (content: any | null) => void;
	open: (map: any, marker: any) => void;
}

export const KakaoMap = () => {
	const [map, setMap] = useState<any>();
	const [userMarker, setUserMarker] = useState<IUserMarker | null>(null);
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

		setMap(newMap);
		setPs(newPs);
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

	// function placesSearchCB(data: any, status: any, pagination: any) {
	function placesSearchCB(data: any, status: any) {
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
				kakao.maps.event.addListener(marker, 'click', function () {
					displayInfowindow(marker, title);
				});
			})(marker, places[i].place_name);
		}
		map.setBounds(newBound);
	}

	function addMarker(position: any, idx: number) {
		let marker = new window.kakao.maps.Marker({
			position: position,
			clickable: true,
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
	}

	// 마커에 클릭이벤트를 등록합니다

	function displayInfowindow(marker: any, title: string) {
		let content = '<div style="padding:5px;z-index:1;color:black;">' + title + '</div>';

		let iwRemoveable = true; // removeable 속성을 ture 로 설정하면 인포윈도우를 닫을 수 있는 x버튼이 표시됩니다

		// 인포윈도우를 생성합니다
		let infowindow = new kakao.maps.InfoWindow({
			content: content,
			removable: iwRemoveable,
		});

		infowindow.setContent(content);
		infowindow.open(map, marker);
	}

	const goCurrentLoc = () => {
		if (userMarker) {
			deleteCurMarkers();
		}
		navigator.geolocation.getCurrentPosition(
			function (position) {
				let currentPos = new window.kakao.maps.LatLng(
					position.coords.latitude,
					position.coords.longitude,
				);
				let message = '<div style="padding:5px;">내 위치</div>';

				displayMarker(currentPos, message);
			},
			// getPosSuccess,
			// () => alert('위치 정보를 가져오는데 실패했습니다.'),
			// {
			// 	enableHighAccuracy: true,
			// 	maximumAge: 20000,
			// 	timeout: 10000,
			// },
		);
		// watchPositionHook();
		// let currentPos = new window.kakao.maps.LatLng(currentLat, currentLng);
		// let message = '<div style="padding:5px;">내 위치</div>';

		// displayMarker(currentPos, message);
	};

	function displayMarker(locPosition: any, message: any) {
		const imageSrc = CurLocImage;
		let imageSize = new window.kakao.maps.Size(40, 40);

		let imgOptions = {
			offset: new window.kakao.maps.Point(20, 0),
		};

		let markerImage = new window.kakao.maps.MarkerImage(imageSrc, imageSize, imgOptions);
		let curLocMarker = new window.kakao.maps.Marker({
			position: locPosition,
			image: markerImage,
		});

		setUserMarker(curLocMarker);

		let iwContent = message; // 인포윈도우에 표시할 내용
		let iwRemoveable = true;

		// 인포윈도우를 생성합니다
		let curLocInfowindow = new kakao.maps.InfoWindow({
			content: iwContent,
			removable: iwRemoveable,
		});

		// 인포윈도우를 마커위에 표시합니다
		curLocInfowindow.open(map, curLocMarker);

		curLocMarker.setMap(map);
		// 지도 중심좌표를 접속위치로 변경합니다
		map.panTo(locPosition);
	}

	function deleteCurMarkers() {
		userMarker?.setMap(null);
	}

	// const getPosSuccess = (pos: GeolocationPosition) => {
	// 	let currentPos = new window.kakao.maps.LatLng(pos.coords.latitude, pos.coords.longitude);
	// 	let message = '<div style="padding:5px;">내 위치</div>';

	// 	displayMarker(currentPos, message);
	// };
	return (
		<>
			<div id='map' style={{ width: '100vw', height: '100vh' }} className='animate-fadeIn'></div>;
			<div className='fixed z-10 w-[65%] h-[35px] top-[5px] left-[5px] flex justify-evenly items-center border border-none rounded-lg bg-slate-100 shadow-inner'>
				<form onSubmit={searchPlaces} className='px-[5px]'>
					<div className='flex justify-evenly items-center'>
						<div className='flex items-center'>
							<div className='mx-[5px] font-["Pretendard-Bold"]'>키워드 : </div>
							<input
								className='w-[60%] rounded-sm border border-none focus:outline-none focus:border-sky-500'
								type='text'
								value={keyword}
								onChange={handleKeywordChange}
							/>
						</div>
						<button type='submit' className='w-[20%] flex justify-center mx-[3px]'>
							<div className='w-[33px] font-["Pretendard-Bold"] rounded-lg bg-lightGreen text-white'>
								검색
							</div>
						</button>
					</div>
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
