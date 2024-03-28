import { useState, useEffect } from 'react';
import LatLngAddStore from '../../../store/LatLngAddStore';
import { BackButton } from '../../../components/BackButton';

// kakao 변수를 전역으로 선언
declare global {
	interface Window {
		kakao: any;
	}
}

export const SetArrival = () => {
	const [map, setMap] = useState<any>();
	const [ps, setPs] = useState<any>();
	const [geocoder, setGeocoder] = useState<any>();
	const [keyword, setKeyword] = useState<string>('');
	const [arrivalMarker, setArrivalMarker] = useState<any>();
	const [address, setAddress] = useState<string>('');

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
		const newPs = new window.kakao.maps.services.Places();

		const newGeocoder = new kakao.maps.services.Geocoder();

		let newAddress = '';

		newGeocoder.coord2Address(
			newMap.getCenter().getLng(),
			newMap.getCenter().getLat(),
			function (result: any, status: any) {
				if (status === kakao.maps.services.Status.OK) {
					console.log(result);

					newAddress = result[0].road_address
						? result[0].road_address.address_name
						: result[0].address.address_name;

					setAddress(newAddress);
				}
			},
		);

		setMap(newMap);
		setPs(newPs);
		setGeocoder(newGeocoder);
		initMarker(newMap, newGeocoder, newAddress);
	}, []);

	const handleKeywordChange = (e: React.ChangeEvent<HTMLInputElement>) => {
		setKeyword(e.target.value);
	};

	const initMarker = (map: any, geocoder: any, address: string) => {
		if (!!arrivalMarker === false) {
			const arriveSrc = 'https://t1.daumcdn.net/localimg/localimages/07/mapapidoc/blue_b.png'; // 도착 마커이미지 주소입니다
			const arriveSize = new kakao.maps.Size(50, 45); // 도착 마커이미지의 크기입니다
			const arriveOption = {
				offset: new kakao.maps.Point(15, 43), // 도착 마커이미지에서 마커의 좌표에 일치시킬 좌표를 설정합니다 (기본값은 이미지의 가운데 아래입니다)
			};

			// 도착 마커 이미지를 생성합니다
			const arriveImage = new kakao.maps.MarkerImage(arriveSrc, arriveSize, arriveOption);

			const arriveDragSrc = 'https://t1.daumcdn.net/localimg/localimages/07/mapapidoc/blue_drag.png'; // 도착 마커의 드래그 이미지 주소입니다
			const arriveDragSize = new kakao.maps.Size(50, 64); // 도착 마커의 드래그 이미지 크기입니다
			const arriveDragOption = {
				offset: new kakao.maps.Point(15, 54), // 도착 마커의 드래그 이미지에서 마커의 좌표에 일치시킬 좌표를 설정합니다 (기본값은 이미지의 가운데 아래입니다)
			};

			// 도착 마커의 드래그 이미지를 생성합니다
			const arriveDragImage = new kakao.maps.MarkerImage(
				arriveDragSrc,
				arriveDragSize,
				arriveDragOption,
			);

			// 도착 마커가 표시될 위치입니다
			// const arrivePosition = new kakao.maps.LatLng(map.getCenter().getLat(), map.getCenter().getLng());
			const arrivePosition = new kakao.maps.LatLng(lat, long);

			// 도착 마커를 생성합니다
			const newArriveMarker = new kakao.maps.Marker({
				map: map, // 도착 마커가 지도 위에 표시되도록 설정합니다
				position: arrivePosition,
				zIndex: 1,
				draggable: true, // 도착 마커가 드래그 가능하도록 설정합니다
				image: arriveImage, // 도착 마커이미지를 설정합니다
			});

			// 도착 마커에 dragstart 이벤트를 등록합니다
			(function (arrivalMarker) {
				kakao.maps.event.addListener(arrivalMarker, 'dragstart', function () {
					// 도착 마커의 드래그가 시작될 때 마커 이미지를 변경합니다
					arrivalMarker.setImage(arriveDragImage);
				});
			})(newArriveMarker);

			// 도착 마커에 dragend 이벤트를 등록합니다
			(function (arrivalMarker, geocoder, address) {
				kakao.maps.event.addListener(arrivalMarker, 'dragend', function () {
					// 도착 마커의 드래그가 종료될 때 마커 이미지를 원래 이미지로 변경합니다
					arrivalMarker.setImage(arriveImage);
					geocoder.coord2Address(
						arrivalMarker.getPosition().getLng(),
						arrivalMarker.getPosition().getLat(),
						function (result: any, status: any) {
							if (status === kakao.maps.services.Status.OK) {
								console.log(result);

								address = result[0].road_address
									? result[0].road_address.address_name
									: result[0].address.address_name;

								setAddress(address);
							}
						},
					);
				});
			})(newArriveMarker, geocoder, address);

			setArrivalMarker(newArriveMarker);
		}
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

	return (
		<>
			<div className='fixed z-10 top-0 w-[100%] h-[15%] flex-col justify-evenly items-end bg-white'>
				<div className='flex'>
					<BackButton />
					<div className='fixed flex justify-center w-[100%] top-[3%] mx-[5px] font-["Pretendard-Bold"] text-[25px]'>
						<div>장소 찾기</div>
					</div>
				</div>
				<form onSubmit={searchPlaces} className='fixed w-[100%] top-[10%] px-[5px] right-0'>
					<div className='w-[100%] flex justify-evenly items-center'>
						<div className='flex items-center'>
							<input
								className='w-[95%] px-[5%] font-["Pretendard-Bold"] rounded-sm border border-lightGray focus:outline-sky-500 focus:border-sky-500'
								type='text'
								value={keyword}
								onChange={handleKeywordChange}
							/>
						</div>
						<button
							type='submit'
							className='flex justify-center w-[15%] bg-[#C4B5FC] border border-none rounded-md'>
							<span className='w-[100%] font-["Pretendard-Bold"] rounded-lg text-white'>검색</span>
						</button>
					</div>
				</form>
			</div>
			<div
				id='map'
				style={{ position: 'fixed', top: '15%', width: '100vw', height: '85vh' }}
				className='animate-fadeIn'></div>
		</>
	);
};
