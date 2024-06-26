import { useEffect, useRef, useState } from 'react';
import { Map } from '../../../components/Map';
import LoadingBus from '../../../assets/loading/loading.gif';
// import UserMarkerSrc from '../../../assets/icon/CurLoc_Img.png';
import CarMarkerSrc from '../../../assets/icon/CarMarker.png';
import StopTrackingMove from '../../../components/StopTrackingMove';
import FootInfoItem from '../../../components/FootInfoItem';
import { formatTime } from '../../../utils/timeUtil';
// import { toPng } from 'html-to-image';
import { useMQTT } from '../../../utils/mqttProvider';
import { MoveStore } from '../../../store/MoveStore';
// import { CarStore } from '../../../store/CarStore';
// import { getCookie, setCookie } from '../../../utils/cookieUtil';
// import { postEndMove } from '../../../apis/MoveApi';
// import { useMutation } from '@tanstack/react-query';
import { useNavigate } from 'react-router-dom';

export const MovingTAISO = () => {
	const navigate = useNavigate();
	const { locationData } = useMQTT();
	const modalRef = useRef<HTMLDialogElement>(null);
	// 시간 상태를 관리합니다. 초기값은 0입니다.
	// const [time, setTime] = useState(0);
	// 스톱워치가 실행 중인지 여부를 관리합니다.
	const CaptureRef = useRef<any>(null);
	const [isMoving, setIsMoving] = useState(true);
	// const [totalDistance, setTotalDistance] = useState(0);
	const polylineRef = useRef<any>(null); // polyline 객체를 저장할 ref
	const userMarkerRef = useRef<any>(null);
	// const carMarkerRef = useRef<any>(null);

	const {
		location: area,
		setLocationList: setAreaList,
		locationList: areaList,
		resetLocationList,
		setTotalDistance,
		totalDistance,
		setTotalTime,
		totalTime,
		time,
		setTime,
		resetTime,
	} = MoveStore();

	const [location, setLocation] = useState({
		center: {
			lat: area[0],
			lng: area[1],
		},
		isLoading: true,
	});

	const [locationList, setLocationList] = useState<any>([]);
	const [copyMap, setCopyMap] = useState<any>(null);

	// const {
	// 	carLocation: carArea,
	// 	setCarLocationList: setCarAreaList,
	// 	resetCarLocationList,
	// } = CarStore();

	// const [carLocation, setCarLocation] = useState({
	// 	center: {
	// 		lat: carArea[0],
	// 		lng: carArea[1],
	// 	},
	// 	isLoading: true,
	// });

	// const [carLocationList, setCarLocationList] = useState<any>([]);

	// const memberInfo = getCookie('member');
	// const { accessToken } = memberInfo;

	// const EndMovemutation = useMutation({
	// 	mutationFn: postEndMove,
	// 	onSuccess: () => {
	// 		setTotalDistance(0),
	// 			resetTime(),
	// 			setTotalTime('00:00:00'),
	// 			resetLocationList(),
	// 			localStorage.removeItem('Move');
	// 		alert('이동기록이 저장되었습니다!');
	// 		navigate('/', { replace: true });
	// 	},
	// });

	const handleCopyMap = (value: any) => {
		setCopyMap(value);
	};

	const handleMoveTrack = () => {
		setIsMoving((pre) => !pre);
	};

	// const stopMove = () => {
	// 	const MoveObj = localStorage.getItem('Move');
	// 	if (MoveObj) {
	// 		if (confirm('목적지까지 무사히 도착하셨나요?')) {
	// 			EndMovemutation.mutate({
	// 				time: totalTime,
	// 				distance: totalDistance,
	// 				spotLists: areaList,
	// 				token: accessToken,
	// 			});
	// 		}
	// 	}
	// };

	const stopMove = () => {
		setIsMoving(false);
		setTotalDistance(0),
			resetTime(),
			setTotalTime('00:00:00'),
			resetLocationList(),
			localStorage.removeItem('User');
		// localStorage.removeItem('Car');
		// resetCarLocationList();
		navigate('/', { replace: true });
	};

	const [iMarker, setIMarker] = useState<any>(null);
	// const [carMarker, setCarMarker] = useState<any>(null);

	useEffect(() => {
		const MoveObj = localStorage.getItem('User');
		// const CarObj = localStorage.getItem('Car');
		if (!MoveObj) {
			alert('실행 중 오류가 발생했습니다. 다시 시도해주세요');
			navigate('/');
		}
	}, []);

	useEffect(() => {
		// const UserMarkerImageSrc = UserMarkerSrc;
		const UserMarkerImageSrc = CarMarkerSrc;
		const UserMarkerImageSize = new window.kakao.maps.Size(40, 40);
		const UserMarkerImgOptions = {
			offset: new window.kakao.maps.Point(20, 20),
		};
		const UserMarkerImage = new window.kakao.maps.MarkerImage(
			UserMarkerImageSrc,
			UserMarkerImageSize,
			UserMarkerImgOptions,
		);
		const CurLocMarker = new window.kakao.maps.Marker({
			image: UserMarkerImage,
		});
		setIMarker(CurLocMarker);

		// const CarImageMarkerSrc = CarMarkerSrc;
		// const CarMarkerImage = new window.kakao.maps.MarkerImage(
		// 	CarImageMarkerSrc,
		// 	UserMarkerImageSize,
		// 	UserMarkerImgOptions,
		// );

		// const CarMarker = new window.kakao.maps.Marker({
		// 	image: CarMarkerImage,
		// });
		// setCarMarker(CarMarker);
	}, []);

	useEffect(() => {
		if (areaList.length > 0) {
			const prevList = areaList.map((item: any) => new window.kakao.maps.LatLng(item.La, item.Ma));
			setLocationList((prev: any) => [...prev, ...prevList]);
		}
	}, []);

	// 위치를 실시간으로 받아오고 로케이션으로 넣어줌
	// Geolocation 아니고 MQTT 데이터 받아서 polyline 성능문제는 없을듯!
	useEffect(() => {
		let watchId: number | null = null;
		// 사용자 위치 추적을 시작하는 함수
		const startLocationTracking = () => {
			// if ('geolocation' in navigator) {
			// 	watchId = navigator.geolocation.watchPosition(
			// 		(position) => {
			// 			const lat = position.coords.latitude;
			// 			const lng = position.coords.longitude;
			// 			console.log(new window.kakao.maps.LatLng(lat, lng));
			// 			setLocation((prev) => ({
			// 				...prev,
			// 				center: { lat, lng },
			// 				isLoading: false,
			// 			}));
			// 			setLocationList((prev: any) => [...prev, new window.kakao.maps.LatLng(lat, lng)]);
			// 			setAreaList(new window.kakao.maps.LatLng(lat, lng));
			// 		},
			// 		(error) => {
			// 			console.log(error);
			// 		},
			// 		{
			// 			enableHighAccuracy: true,
			// 			maximumAge: 1000,
			// 			timeout: 2000,
			// 		},
			// 	);
			if (!!locationData) {
				const lat = locationData.latitude;
				const lng = locationData.longitude;
				console.log(new window.kakao.maps.LatLng(lat, lng));
				setLocation((prev) => ({
					...prev,
					center: { lat, lng },
					isLoading: false,
				}));
				setLocationList((prev: any) => [...prev, new window.kakao.maps.LatLng(lat, lng)]);
				setAreaList(new window.kakao.maps.LatLng(lat, lng));
			} else {
				console.log('mqtt data is not available.');
			}
		};

		// // 차량의 위치 추적을 시작하는 함수
		// const startCarTracking = () => {
		// 	// MQTT로 데이터 받아오는 부분
		// 	const lat = locationData?.latitude;
		// 	const lng = locationData?.longitude;
		// 	// const lat = 36.1287694;
		// 	// const lng = 128.3296222;
		// 	// setCarLocation((prev) => ({
		// 	// 	...prev,
		// 	// 	center: { lat, lng },
		// 	// 	isLoading: false,
		// 	// }));
		// 	setCarLocation({
		// 		center: { lat, lng },
		// 		isLoading: false,
		// 	});
		// 	setCarLocationList((prev: any) => [...prev, new window.kakao.maps.LatLng(lat, lng)]);
		// 	setCarAreaList(new window.kakao.maps.LatLng(lat, lng));
		// };

		// `isMoving` 상태가 true일 때만 위치 추적을 시작합니다.
		if (isMoving) {
			startLocationTracking();
			// startCarTracking();
		}

		// 클린업 함수에서는 위치 추적을 중단합니다.
		return () => {
			if (watchId !== null) {
				navigator.geolocation.clearWatch(watchId);
			}
		};
	}, [isMoving, setLocationList, locationData]); // `isMoving` 상태가 변경될 때마다 이 useEffect를 다시 실행합니다.

	// 폴리라인 그리는 것
	useEffect(() => {
		if (copyMap && locationList.length > 3 && window.kakao.maps) {
			if (!polylineRef.current) {
				const polyline = new window.kakao.maps.Polyline({
					path: locationList,
					strokeWeight: 7.5,
					strokeColor: '#8F72F9',
					strokeOpacity: 0.7,
					strokeStyle: 'solid',
				});
				polyline.setMap(copyMap);
				polylineRef.current = polyline;
			} else {
				polylineRef.current.setPath(locationList);
			}
		}
		return () => {
			resetLocationList();
		};
	}, [locationList, location.center, copyMap]);

	// 마커
	useEffect(() => {
		const userMarkerPosition = new window.kakao.maps.LatLng(location.center.lat, location.center.lng);
		// const carMarkerPosition = new window.kakao.maps.LatLng(
		// 	carLocation.center.lat,
		// 	carLocation.center.lng,
		// );
		// const bounds = new window.kakao.maps.LatLngBounds(userMarkerPosition, carMarkerPosition);
		if (copyMap && window.kakao.maps) {
			// if (userMarkerRef.current && carMarkerRef.current) {
			if (userMarkerRef.current) {
				userMarkerRef.current.setPosition(userMarkerPosition);
				// carMarkerRef.current.setPosition(carMarkerPosition);
				// copyMap.setCenter(userMarkerPosition);
				// copyMap.setBounds(bounds);
				copyMap.setCenter(userMarkerPosition);
			} else {
				iMarker.setPosition(userMarkerPosition);
				// carMarker.setPosition(carMarkerPosition);
				iMarker.setMap(copyMap);
				// carMarker.setMap(copyMap);
				userMarkerRef.current = iMarker;
				// carMarkerRef.current = carMarker;
			}
		}
		// return () => {
		//   userMarkerRef.current.setMap(null);
		// };
	}, [location.center, copyMap]);

	// 스톱 워치
	useEffect(() => {
		let interval: any;

		if (isMoving) {
			// 스톱워치가 실행 중일 때 1초마다 time 상태를 업데이트합니다.
			interval = setInterval(() => {
				setTime();
			}, 1000);
		}

		// 컴포넌트가 언마운트되거나 isRunning 상태가 변경될 때 인터벌을 정리합니다.
		return () => clearInterval(interval);
	}, [isMoving, setTime]);

	// 거리 계산
	useEffect(() => {
		if (polylineRef.current) {
			// 폴리라인의 총 길이(거리)를 계산하여 상태를 업데이트합니다.
			const distanceM = polylineRef.current.getLength();
			const distanceKM = distanceM / 1000;
			setTotalDistance(distanceKM);
		}
		setTotalTime(formatTime(time));
	}, [locationList, setTotalDistance, setTotalTime, time]); // locationList가 변경될 때마다 이 useEffect를 실행합니다.

	// 이동 중에 화면이 꺼지지 않도록 하는 기능
	useEffect(() => {
		let wakeLock: any = null;

		async function requestWakeLock() {
			if ('wakeLock' in navigator) {
				try {
					wakeLock = await navigator.wakeLock.request('screen');
					console.log('Screen Wake Lock activated');
				} catch (err) {
					console.error(`something went wrong`);
				}
			}
		}

		requestWakeLock();

		return () => {
			if (wakeLock !== null) {
				wakeLock.release().then(() => {
					wakeLock = null;
					console.log('Screen Wake Lock released');
				});
			}
		};
	}, []);

	// const htmlToImageConvert = () => {
	// 	toPng(CaptureRef.current, { cacheBust: false })
	// 		.then((dataUrl) => {
	// 			// CORS 프록시 URL 추가
	// 			const proxyUrl = 'https://cors-anywhere.herokuapp.com/';
	// 			// 실제 이미지 URL
	// 			const imageUrl = dataUrl;
	// 			// 프록시를 사용하여 이미지 다운로드 링크 생성
	// 			const proxiedImageUrl = proxyUrl + imageUrl;

	// 			const link = document.createElement('a');
	// 			link.download = 'my-image-name.png';
	// 			link.href = proxiedImageUrl;
	// 			link.click();
	// 		})
	// 		.catch((err) => {
	// 			console.log(err);
	// 		});
	// };

	const checkIsMoving = () => {
		setIsMoving(true);
	};

	const handlecCancel = () => {
		navigate('/');
	};

	let modal = (
		<>
			{/* Open the modal using document.getElementById('ID').showModal() method */}
			<dialog ref={modalRef} id='my_modal_2' className='modal'>
				<div className='modal-box'>
					<h3 className='font-bold text-lg'>탑승확인</h3>
					<p className='py-4'>현재 차량에 탑승중이신가요 ?</p>
					<div className='modal-action'>
						<form method='dialog'>
							{/* if there is a button in form, it will close the modal */}
							<button className='btn mx-5' onClick={handlecCancel}>
								아니요
							</button>
							<button className='btn bg-[#C4B5FC] text-white' onClick={checkIsMoving}>
								네
							</button>
						</form>
					</div>
				</div>
			</dialog>
		</>
	);

	return (
		<>
			<div className='flex flex-col items-center w-[100%] h-[100%] '>
				{location.isLoading ? (
					<div className='flex justify-center items-center w-[100%] h-[100%]'>
						<img src={LoadingBus} alt='LOADING' />
					</div>
				) : (
					<div ref={CaptureRef} className='w-[100%] h-[80%]'>
						<Map
							width='100%'
							height='100%'
							lat={location.center.lat}
							lng={location.center.lng}
							handleCopyMap={handleCopyMap}
						/>
					</div>
				)}

				<div className='fixed bottom-0 flex flex-col justify-evenly items-center mt-[3%] rounded-lg bg-white w-[100%] h-[23%] z-10'>
					<div className='flex justify-evenly items-center w-[100%]'>
						<div className='flex flex-col justify-center items-center w-[40%]'>
							<div className='text-[20px] font-["Pretendard-Bold"]'>{totalTime}</div>
							<div className='text-'>이동 시간</div>
						</div>
						<div className='w-[40%]'>
							<FootInfoItem title='이동거리(km)' value={totalDistance?.toFixed(2)} />
						</div>
					</div>
					<div className='flex justify-center items-center w-[70%] h-[30%]'>
						<StopTrackingMove isMoving={isMoving} handleMoveTrack={handleMoveTrack} stopMove={stopMove} />
					</div>
				</div>

				{/* <button onClick={htmlToImageConvert}>이동기록저장</button> */}
			</div>
		</>
	);
};
