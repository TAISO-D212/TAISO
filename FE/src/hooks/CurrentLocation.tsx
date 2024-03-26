import LatLngAddStore from '../store/LatLngAddStore.ts';

function useCurrentLocation() {
	const { setLatLngAdd } = LatLngAddStore((state) => state);

	if (navigator.geolocation) {
		navigator.geolocation.getCurrentPosition(success, error);
	}

	function success(position: GeolocationPosition) {
		localStorage.setItem('latitude', position.coords.latitude.toString());
		localStorage.setItem('longitude', position.coords.longitude.toString());
		setLatLngAdd(
			position.coords.latitude,
			position.coords.longitude,
			localStorage.getItem('address') as string,
		);
	}
	function error() {
		alert('위치받기 실패');
	}
}

export default useCurrentLocation;
