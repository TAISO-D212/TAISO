import CurrentLoc from "../../../assets/image/CurrentLoc.png"

interface ISetCurrentLoc {
  map: any;
  marker: any;
  setMap: (map: any) => void;
}

export const SetCurrentLoc = ({ map, marker }: ISetCurrentLoc) => {

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

  const goCurrentLoc = () => {
    navigator.geolocation.getCurrentPosition(
      getPosSuccess,
      () => alert("위치 정보를 가져오는데 실패했습니다."),
      {
        enableHighAccuracy: true,
        maximumAge: 30000,
        timeout: 27000,
      }
  )}
  
  return (
    <>
      <div className="fixed w-[50px] h-50px] bottom-[90px] right-[9px] z-10 flex flex-col justify-center items-center" onClick={goCurrentLoc}>
        <img src={CurrentLoc} alt="HOME_TAB"/>
      </div>
    </>
  )

}