import { useState, useEffect, useRef } from 'react';
import LatLngAddStore from '../../../store/LatLngAddStore';
import { HomeBackButton } from '../../../components/HomeBackButton';
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

export const SetArrivalByMap = () => {
	const navigate = useNavigate();
	const [map, setMap] = useState<any>();
	const [ps, setPs] = useState<any>();
	const [geocoder, setGeocoder] = useState<any>();
	const [keyword, setKeyword] = useState<string>('');
	const [arriveMarker, setArriveMarker] = useState<any>();
	const [name, setName] = useState<string>('');
	const [address, setAddress] = useState<string>('');
	const [endLat, setEndLat] = useState<number | null>();
	const [endLng, setEndLng] = useState<number | null>();
	const modalRef = useRef<HTMLDialogElement>(null);

	const openModal = () => {
		if (modalRef.current) {
			modalRef.current.showModal();
		}
	};

	const closeModal = () => {
		if (modalRef.current) {
			modalRef.current.close();
		}
	};

	const { setEndAddress, setEndLatitude, setEndLongitude, setEndBookmarkId } = NewReservationStore();

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
					setEndLat(newMap.getCenter().getLat());
					setEndLng(newMap.getCenter().getLng());
				}
			},
		);

		setMap(newMap);
		setPs(newPs);
		setGeocoder(newGeocoder);
		initMarker(newMap, newGeocoder, newAddress);
	}, []);

	const handleSetEndLoc = () => {
		setEndBookmarkId(null);
		setEndAddress(address);
		setEndLatitude(endLat);
		setEndLongitude(endLng);
		navigate('/reservation/new');
	};

	const initMarker = (map: any, geocoder: any, address: string) => {
		if (!!arriveMarker === false) {
			const arriveSrc = 'https://t1.daumcdn.net/localimg/localimages/07/mapapidoc/blue_b.png'; // 출발 마커이미지의 주소입니다
			const arriveSize = new kakao.maps.Size(50, 45); // 출발 마커이미지의 크기입니다
			const arriveOption = {
				offset: new kakao.maps.Point(15, 43), // 출발 마커이미지에서 마커의 좌표에 일치시킬 좌표를 설정합니다 (기본값은 이미지의 가운데 아래입니다)
			};

			// 도착 마커 이미지를 생성합니다
			const arriveImage = new kakao.maps.MarkerImage(arriveSrc, arriveSize, arriveOption);

			const arriveDragSrc = 'https://t1.daumcdn.net/localimg/localimages/07/mapapidoc/blue_drag.png'; // 출발 마커의 드래그 이미지 주소입니다
			const arriveDragSize = new kakao.maps.Size(50, 64); // 출발 마커의 드래그 이미지 크기입니다
			const arriveDragOption = {
				offset: new kakao.maps.Point(15, 54), // 출발 마커의 드래그 이미지에서 마커의 좌표에 일치시킬 좌표를 설정합니다 (기본값은 이미지의 가운데 아래입니다)
			};

			// 출발 마커의 드래그 이미지를 생성합니다
			const arriveDragImage = new kakao.maps.MarkerImage(
				arriveDragSrc,
				arriveDragSize,
				arriveDragOption,
			);

			// 출발 마커가 표시될 위치입니다
			const arrivePosition = new kakao.maps.LatLng(lat, long);

			// 출발 마커를 생성합니다
			const newArriveMarker = new kakao.maps.Marker({
				map: map, // 출발 마커가 지도 위에 표시되도록 설정합니다
				position: arrivePosition,
				zIndex: 1,
				draggable: true, // 출발 마커가 드래그 가능하도록 설정합니다
				image: arriveImage, // 출발 마커이미지를 설정합니다
			});

			// 출발 마커에 dragstart 이벤트를 등록합니다
			(function (arriveMarker) {
				kakao.maps.event.addListener(arriveMarker, 'dragstart', function () {
					// 출발 마커의 드래그가 시작될 때 마커 이미지를 변경합니다
					arriveMarker.setImage(arriveDragImage);
				});
			})(newArriveMarker);

			// 출발 마커에 dragend 이벤트를 등록합니다
			(function (arriveMarker, geocoder, address) {
				kakao.maps.event.addListener(arriveMarker, 'dragend', function () {
					// 출발 마커의 드래그가 종료될 때 마커 이미지를 원래 이미지로 변경합니다
					arriveMarker.setImage(arriveImage);
					geocoder.coord2Address(
						arriveMarker.getPosition().getLng(),
						arriveMarker.getPosition().getLat(),
						function (result: any, status: any) {
							if (status === kakao.maps.services.Status.OK) {
								console.log(result);

								address = result[0].road_address
									? result[0].road_address.address_name
									: result[0].address.address_name;

								setAddress(address);
								setEndLat(arriveMarker.getPosition().getLat());
								setEndLng(arriveMarker.getPosition().getLng());
							}
						},
					);
				});
			})(newArriveMarker, geocoder, address);

			setArriveMarker(newArriveMarker);
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
			name,
			latitude: endLat,
			longitude: endLng,
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

	const handleFavoriteName = (e: React.ChangeEvent<HTMLInputElement>) => {
		setName(e.target.value as string);
	};

	let modal = (
		<>
			{/* Open the modal using document.getElementById('ID').showModal() method */}
			<dialog ref={modalRef} id='my_modal_1' className='modal'>
				<div className='modal-box'>
					<h3 className='font-bold text-lg'>즐겨찾기 추가</h3>
					<p className='py-4'>현재 위치의 이름을 추가해주세요.</p>
					<input
						type='text'
						placeholder='즐겨찾기 이름'
						className='input input-bordered w-full'
						onChange={handleFavoriteName}
					/>
					<div className='modal-action'>
						<form method='dialog'>
							{/* if there is a button in form, it will close the modal */}
							<button className='btn mx-5'>취소</button>
							<button className='btn bg-[#C4B5FC] text-white' onClick={fetchFavorite}>
								추가
							</button>
						</form>
					</div>
				</div>
			</dialog>
		</>
	);

	return (
		<>
			{/* TODO : 지도에서 찾기 토글 버튼으로 수정하기 -> 
		지도에서 찾기 버튼 클릭
		즐겨찾기 목록 안보이고, 지도 활성화
		즐겨찾기 목록에서 즐겨찾기 삭제 만들기 (즐겨찾기 DELETE 방식으로 BE에 전달하기)
		현위치 즐겨찾기 추가 만들기 (즐겨찾기 POST 방식으로 BE에 전달하기)
		*/}
			{modal}
			<div className='fixed z-10 top-0 w-[100%] h-[15%] flex-col justify-evenly items-end bg-white'>
				<div className='flex'>
					<HomeBackButton />
					<div className='fixed flex justify-center w-[100%] top-[3%] mx-[5px] font-["Pretendard-Bold"] text-[25px]'>
						<div>장소 찾기</div>
					</div>
				</div>
				<form onSubmit={searchPlaces} className='fixed w-[100%] top-[10%] px-[5px] right-0'>
					<div className='w-[100%] flex justify-evenly items-center'>
						<div className='flex items-center'>
							<input
								className='w-[100%] pl-[10%] py-1 font-["Pretendard-Bold"] rounded-md border-2 border-slate-300 placeholder:text-slate-400 focus:outline-sky-500 focus:border-sky-500'
								type='text'
								placeholder='도착지를 입력해 주세요.'
								value={keyword}
								onChange={handleKeywordChange}
							/>
						</div>
						<button
							type='submit'
							className='flex justify-center py-[5px] w-[15%] bg-[#C4B5FC] border border-none rounded-md'>
							<span className='w-[100%] font-["Pretendard-Bold"] rounded-sm text-white'>검색</span>
						</button>
					</div>
				</form>
			</div>
			<div className='fixed z-10 bottom-0 w-[100%] h-[20%] flex flex-col justify-center items-center bg-white'>
				<div className='w-[100%] font-["Pretendard-Bold"] flex justify-center items-center text-[20px] my-3'>
					{address}
				</div>
				<div className='w-[65%] mb-3' onClick={openModal}>
					<StarIcon sx={{ color: '#fcfc9c' }} />
					<span>현 위치 즐겨찾기 추가 {'>'}</span>
				</div>
				<div
					className='w-[70%] h-[40%] mb-2 flex justify-center items-center bg-[#3422F2] rounded-full'
					onClick={handleSetEndLoc}>
					<span className='flex justify-center w-[100%] font-["Pretendard-Bold"] text-[25px] text-white'>
						도착지 설정
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
