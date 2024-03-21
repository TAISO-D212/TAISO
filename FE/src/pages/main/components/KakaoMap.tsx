/* eslint-disable @typescript-eslint/no-explicit-any */
import { useState, useEffect } from 'react';
import CurrentLoc from "../../../assets/image/CurrentLoc.png"
import { LatLngAddStore } from '../../../store/LatLngAddStore';

declare global {
  interface Window {
    kakao: any;
  }
}

export const KakaoMap= () => {
  const [map, setMap] = useState<any>();
  const [marker, setMarker] = useState<any>();

  const { currentLat, currentLng } = LatLngAddStore((state) => state);
  const latitude = currentLat;
  const longitude = currentLng;
    
  // 1) 카카오맵 불러오기
  useEffect(() => {
    window.kakao.maps.load(() => {
      const container = document.getElementById("map");
      const options = {
        center: new window.kakao.maps.LatLng(latitude, longitude),
        level: 3,
      };
      
      setMap(new window.kakao.maps.Map(container, options));
      setMarker(new window.kakao.maps.Marker());
    });
  }, []);
  
  // 2) 현재 위치 함수
  const goCurrentLoc = () => {
    navigator.geolocation.getCurrentPosition(
      getPosSuccess,
      () => alert("위치 정보를 가져오는데 실패했습니다."),
      {
        enableHighAccuracy: true,
        maximumAge: 30000,
        timeout: 27000,
      }
    );
  }
  
  // 3) 정상적으로 현재위치 가져올 경우 실행
  const getPosSuccess = (pos: GeolocationPosition) => {
    // 현재 위치(위도, 경도) 가져온다.
    let currentPos = new window.kakao.maps.LatLng(
      pos.coords.latitude, // 위도
      pos.coords.longitude // 경도
    );
    // 지도를 이동 시킨다.
    map.panTo(currentPos);

    // 기존 마커를 제거하고 새로운 마커를 넣는다.
    marker.setMap(null);
    marker.setPosition(currentPos);
    marker.setMap(map);
  };

  return (<>
  <div id="map" style={{ width: '100vw', height: '100vh' }} className='animate-fadeIn'></div>;
  <div className="fixed w-[50px] h-50px] bottom-[90px] right-[9px] z-10 flex flex-col justify-center items-center" onClick={goCurrentLoc}>
        <img src={CurrentLoc} alt="HOME_TAB"/>
      </div>
  </>
)}