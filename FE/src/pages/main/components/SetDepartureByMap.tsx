import { useState, useEffect } from 'react';
import LatLngAddStore from '../../../store/LatLngAddStore';
import { BackButton } from '../../../components/BackButton';
import { BookmarkInputType } from '../../../interfaces/Bookmark';
import { useNavigate } from 'react-router-dom';
import NewReservationStore from '../../../store/NewReservationStore';
import StarIcon from '@mui/icons-material/Star';
import { addBookmark } from '../../../apis/bookmarkApi';

// kakao 변수를 전역으로 선언
declare global {
	interface Window {
		kakao: any;
	}
}

export const SetDepartureByMap = () => {
	const navigate = useNavigate();
	const [map, setMap] = useState<any>();
	const [ps, setPs] = useState<any>();
	const [geocoder, setGeocoder] = useState<any>();
	const [keyword, setKeyword] = useState<string>('');
	const [startMarker, setStartMarker] = useState<any>();
	const [address, setAddress] = useState<string>('');
	const [startLat, setStartLat] = useState<number | null>();
	const [startLng, setStartLng] = useState<number | null>();

	const { setStartAddress, setStartLatitude, setStartLongitude, setStartPlaceId } =
		NewReservationStore();

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
					setStartLat(newMap.getCenter().getLat());
					setStartLng(newMap.getCenter().getLng());
				}
			},
		);

		setMap(newMap);
		setPs(newPs);
		setGeocoder(newGeocoder);
		initMarker(newMap, newGeocoder, newAddress);
	}, []);

	const handleSetStartLoc = () => {
		setStartPlaceId(null);
		setStartAddress(address);
		setStartLatitude(startLat);
		setStartLongitude(startLng);
		navigate('/reservation/new');
	};

	const initMarker = (map: any, geocoder: any, address: string) => {
		if (!!startMarker === false) {
			const startSrc = 'https://t1.daumcdn.net/localimg/localimages/07/mapapidoc/red_b.png'; // 출발 마커이미지의 주소입니다
			const startSize = new kakao.maps.Size(50, 45); // 출발 마커이미지의 크기입니다
			const startOption = {
				offset: new kakao.maps.Point(15, 43), // 출발 마커이미지에서 마커의 좌표에 일치시킬 좌표를 설정합니다 (기본값은 이미지의 가운데 아래입니다)
			};

			// 출발 마커 이미지를 생성합니다
			const startImage = new kakao.maps.MarkerImage(startSrc, startSize, startOption);

			const startDragSrc = 'https://t1.daumcdn.net/localimg/localimages/07/mapapidoc/red_drag.png'; // 출발 마커의 드래그 이미지 주소입니다
			const startDragSize = new kakao.maps.Size(50, 64); // 출발 마커의 드래그 이미지 크기입니다
			const startDragOption = {
				offset: new kakao.maps.Point(15, 54), // 출발 마커의 드래그 이미지에서 마커의 좌표에 일치시킬 좌표를 설정합니다 (기본값은 이미지의 가운데 아래입니다)
			};

			// 출발 마커의 드래그 이미지를 생성합니다
			const startDragImage = new kakao.maps.MarkerImage(startDragSrc, startDragSize, startDragOption);

			// 출발 마커가 표시될 위치입니다
			const startPosition = new kakao.maps.LatLng(lat, long);

			// 출발 마커를 생성합니다
			const newStartMarker = new kakao.maps.Marker({
				map: map, // 출발 마커가 지도 위에 표시되도록 설정합니다
				position: startPosition,
				zIndex: 1,
				draggable: true, // 출발 마커가 드래그 가능하도록 설정합니다
				image: startImage, // 출발 마커이미지를 설정합니다
			});

			// 출발 마커에 dragstart 이벤트를 등록합니다
			(function (startMarker) {
				kakao.maps.event.addListener(startMarker, 'dragstart', function () {
					// 출발 마커의 드래그가 시작될 때 마커 이미지를 변경합니다
					startMarker.setImage(startDragImage);
				});
			})(newStartMarker);

			// 출발 마커에 dragend 이벤트를 등록합니다
			(function (startMarker, geocoder, address) {
				kakao.maps.event.addListener(startMarker, 'dragend', function () {
					// 출발 마커의 드래그가 종료될 때 마커 이미지를 원래 이미지로 변경합니다
					startMarker.setImage(startImage);
					geocoder.coord2Address(
						startMarker.getPosition().getLng(),
						startMarker.getPosition().getLat(),
						function (result: any, status: any) {
							if (status === kakao.maps.services.Status.OK) {
								console.log(result);

								address = result[0].road_address
									? result[0].road_address.address_name
									: result[0].address.address_name;

								setAddress(address);
								setStartLat(startMarker.getPosition().getLat());
								setStartLng(startMarker.getPosition().getLng());
							}
						},
					);
				});
			})(newStartMarker, geocoder, address);

			setStartMarker(newStartMarker);
		}
	};

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

	const fetchFavorite = () => {
		const favoriteObj: BookmarkInputType = {
			name: address,
			latitude: startLat,
			longitude: startLng,
			address: address,
		};

		// 즐겨찾기 추가
		addBookmark(favoriteObj).then((res) => {
			if (res) {
				alert('즐겨찾기에 추가되었습니다.');
			}
			console.log(res);
		});
	};

	return (
		<>
			{/* TODO : 지도에서 찾기 토글 버튼으로 수정하기 -> 
		지도에서 찾기 버튼 클릭
		즐겨찾기 목록 안보이고, 지도 활성화
		즐겨찾기 목록에서 즐겨찾기 삭제 만들기 (즐겨찾기 DELETE 방식으로 BE에 전달하기)
		현위치 즐겨찾기 추가 만들기 (즐겨찾기 POST 방식으로 BE에 전달하기)
		*/}
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
								className='w-[95%] px-[5%] font-["Pretendard-Bold"] rounded-md border-2 border-slate-300 placeholder:text-slate-400 focus:outline-sky-500 focus:border-sky-500'
								type='text'
								placeholder='출발지를 입력해 주세요.'
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
			<div className='fixed z-10 bottom-0 w-[100%] h-[15%] flex flex-col justify-center items-center bg-white'>
				<div className='w-[100%] font-["Pretendard-Bold"] flex justify-center items-center text-[20px] my-3'>
					{address}
				</div>
				<div className='w-[65%] mb-3' onClick={fetchFavorite}>
					<StarIcon sx={{ color: '#fcfc9c' }} />
					<span>현 위치 즐겨찾기 추가 {'>'}</span>
				</div>
				<div
					className=' w-[70%] h-[40%] flex justify-center items-center bg-[#3422F2] rounded-full'
					onClick={handleSetStartLoc}>
					<span className='flex justify-center w-[100%] font-["Pretendard-Bold"] text-[25px] text-white'>
						출발지 설정
					</span>
				</div>
			</div>
			<div
				id='map'
				style={{ position: 'fixed', top: '15%', width: '100vw', height: '70vh' }}
				className='animate-fadeIn'></div>
		</>
	);
};
