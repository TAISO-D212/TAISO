import { useEffect, useRef } from 'react';

interface IMapBoxProps {
	width: string;
	height: string;
	lat: number;
	lng: number;
	handleCopyMap: (value: any) => void;
}

export const Map = ({ width, height, lat, lng, handleCopyMap }: IMapBoxProps) => {
	const mapContainer = useRef<HTMLDivElement>(null);
	useEffect(() => {
		const options = {
			//지도를 생성할 때 필요한 기본 옵션
			center: new window.kakao.maps.LatLng(lat, lng), //지도의 중심좌표.
			level: 3, //지도의 레벨(확대, 축소 정도)
		};
		const map = new window.kakao.maps.Map(mapContainer.current, options); //지도 생성 및 객체 리턴
		handleCopyMap(map);
	}, []);

	return <div ref={mapContainer} id='map' style={{ width: width, height: height }} />;
};
