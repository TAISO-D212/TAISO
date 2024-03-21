/* eslint-disable @typescript-eslint/no-explicit-any */
import { MutableRefObject, useEffect, useRef } from 'react';
import { BottomNav } from './components/BottomNav';

declare global {
  interface Window {
    kakao: any;
  }
}

export const MainPage= () => {
  const mapRef = useRef<HTMLElement | null>(null);

  const initMap = () => {
    const container = document.getElementById('map');
    const options = {
      center: new window.kakao.maps.LatLng(37.483034, 126.902435),
      level: 2,
    };

    const map = new window.kakao.maps.Map(container as HTMLElement, options);
    // eslint-disable-next-line @typescript-eslint/no-explicit-any
    (mapRef as MutableRefObject<any>).current = map;
  };

  useEffect(() => {
    window.kakao.maps.load(() => initMap());
  }, [mapRef]);

  return (<>
  <div id="map" style={{ width: '100vw', height: '100vh' }} className='animate-fadeIn'></div>;
  <BottomNav/>
  </>
)}