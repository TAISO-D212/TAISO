import React, { useState, useEffect, useRef } from 'react';
import { Map, MapTypeId, MapMarker, Roadview } from 'react-kakao-maps-sdk';
import LatLngAddStore from '../../../store/LatLngAddStore';
import CurLocMarker from '../../../assets/image/CurrentLoc.png';

interface Center {
	lat: number;
	lng: number;
}

export const KakaoMap: React.FC = () => {
	const { currentLat, currentLng } = LatLngAddStore((state) => state);
	const latitude: number = currentLat;
	const longitude: number = currentLng;
	const [isAtive, setIsAtive] = useState<boolean>(false);
	const [isVisible, setIsVisible] = useState<boolean>(false);
	const [isOpen, setIsOpen] = useState<boolean>(false);
	const mapRef = useRef<kakao.maps.Map>(null);
	const roadviewRef = useRef<any>(null);

	const [curLoc, setCurLoc] = useState<{
		center: Center;
		errMsg: string | null;
		isLoading: boolean;
	}>({
		center: {
			lat: latitude,
			lng: longitude,
		},
		errMsg: null,
		isLoading: true,
	});

	const [center, setCenter] = useState<Center>({
		lat: latitude,
		lng: longitude,
	});

	useEffect(() => {
		if (navigator.geolocation) {
			// GeoLocation을 이용해서 접속 위치를 얻어옵니다
			navigator.geolocation.getCurrentPosition(
				(position) => {
					setCurLoc((prev) => ({
						...prev,
						center: {
							lat: position.coords.latitude, // 위도
							lng: position.coords.longitude, // 경도
						},
						isLoading: false,
					}));
				},
				(err) => {
					setCurLoc((prev) => ({
						...prev,
						errMsg: err.message,
						isLoading: false,
					}));
				},
			);
		} else {
			// HTML5의 GeoLocation을 사용할 수 없을때 마커 표시 위치와 인포윈도우 내용을 설정합니다
			setCurLoc((prev) => ({
				...prev,
				errMsg: 'geolocation을 사용할수 없어요..',
				isLoading: false,
			}));
		}
	}, []);

	useEffect(() => {
		const map = mapRef.current;
		const roadview = roadviewRef.current;
		if (roadview && map) {
			roadview.relayout();
			map.relayout();
			map.setCenter(new kakao.maps.LatLng(center.lat, center.lng));
		}
	}, [center, isAtive]);

	return (
		<div
			style={{
				display: 'flex',
				flexDirection: 'column',
				position: 'relative',
				width: '100%',
				height: '100%',
			}}>
			<div
				style={{
					position: 'relative',
					width: isVisible ? '100%' : '0',
					height: isVisible ? '50%' : '0',
					overflow: 'hidden',
				}}>
				<Roadview // 로드뷰를 표시할 Container
					position={{ ...center, radius: 50 }}
					style={{
						// 지도의 크기
						width: '100%',
						height: '100%',
					}}
					onPositionChanged={(rv) => {
						setCenter({
							lat: rv.getPosition().getLat(),
							lng: rv.getPosition().getLng(),
						});
					}}
					onPanoidChange={() => {
						isAtive && setIsVisible(true);
					}}
					onErrorGetNearestPanoId={() => {
						setIsVisible(false);
					}}
					ref={roadviewRef}>
					<div id='close' title='로드뷰닫기' onClick={() => setIsVisible(false)}>
						<span className='img'></span>
					</div>
				</Roadview>
			</div>
			<RoadviewWithMapButtonStyle />
			<Map // 로드뷰를 표시할 Container
				id='map'
				center={center}
				style={{
					// 지도의 크기
					width: '100%',
					height: !isVisible ? '100%' : '50%',
				}}
				level={3}
				ref={mapRef}>
				{isAtive && (
					<>
						<MapTypeId type={kakao.maps.MapTypeId.ROADVIEW} />
						<MapMarker
							position={center}
							draggable={true}
							onDragEnd={(marker) => {
								setCenter({
									lat: marker.getPosition().getLat(),
									lng: marker.getPosition().getLng(),
								});
							}}
							image={{
								src: 'https://t1.daumcdn.net/localimg/localimages/07/2018/pc/roadview_minimap_wk_2018.png',
								size: { width: 26, height: 46 },
								options: {
									spriteSize: { width: 1666, height: 168 },
									spriteOrigin: { x: 705, y: 114 },
									offset: { x: 13, y: 46 },
								},
							}}
						/>
					</>
				)}
				{!curLoc.isLoading && (
					<MapMarker
						position={curLoc.center}
						image={{
							src: CurLocMarker,
							size: { width: 40, height: 40 },
							options: {
								offset: { x: 20, y: 5 },
							},
						}}
						clickable={true}
						onClick={() => setIsOpen(true)}>
						{/* <div style={{ padding: '5px', color: '#000' }}>
							{curLoc.errMsg ? curLoc.errMsg : '내 위치'}
						</div> */}
						{isOpen && (
							<div style={{ minWidth: '150px' }}>
								<img
									alt='close'
									width='14'
									height='13'
									src='https://t1.daumcdn.net/localimg/localimages/07/mapjsapi/2x/bt_close.gif'
									style={{
										position: 'absolute',
										right: '5px',
										top: '5px',
										cursor: 'pointer',
									}}
									onClick={() => setIsOpen(false)}
								/>
								<div style={{ padding: '5px', color: '#000' }}>내 위치</div>
							</div>
						)}
					</MapMarker>
				)}
			</Map>

			<div
				id='roadviewControl'
				className={isAtive ? 'active' : ''}
				onClick={() => {
					setIsVisible(true);
					setIsAtive(!isAtive);
				}}>
				<span className='img'></span>
			</div>
		</div>
	);
};

const RoadviewWithMapButtonStyle: React.FC = () => (
	<style>{`
    #container {overflow:hidden;height:300px;position:relative;}
    #rvWrapper {width:50%;height:300px;top:0;right:0;position:absolute;z-index:0;}
    #container.view_roadview #mapWrapper {width: 50%;}
    #roadviewControl {position:absolute;top:5px;right:5px;width:42px;height:42px;z-index: 1;cursor: pointer; background: url(https://t1.daumcdn.net/localimg/localimages/07/2018/pc/common/img_search.png) 0 -450px no-repeat;}
    #roadviewControl.active {background-position:0 -350px;}
    #close {position: absolute;padding: 4px;top: 5px;left: 5px;cursor: pointer;background: #fff;border-radius: 4px;border: 1px solid #c8c8c8;box-shadow: 0px 1px #888;}
    #close .img {display: block;background: url(https://t1.daumcdn.net/localimg/localimages/07/mapapidoc/rv_close.png) no-repeat;width: 14px;height: 14px;}
    `}</style>
);
