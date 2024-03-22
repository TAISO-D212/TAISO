import { LatLngAddStore } from '../store/LatLngAddStore.ts'; 
import { distance } from '../utils/calc.ts'

interface PositionOptions {
    coords: {
        latitude: number;
        longitude: number;
    };
}

interface Options {
    enableHighAccuracy: boolean;
    timeout: number;
    maximumAge: number;
}

export const watchPositionHook = () => {
    const { currentLat, currentLng } = LatLngAddStore((state) => state);

    const success = (pos: PositionOptions) => {
        const d = distance(currentLat, currentLng, pos.coords.latitude, pos.coords.longitude);
        if (d > 0.1) {
            localStorage.setItem('latitude', pos.coords.latitude.toString());
            localStorage.setItem('longitude', pos.coords.longitude.toString());
            LatLngAddStore.setState({
                currentLat: pos.coords.latitude,
                currentLng: pos.coords.longitude,
            });
        }
    }

    const error = (err: { code: number; message: string }) => {
        alert('ERROR(' + err.code + '): ' + err.message);
    }

    const options: Options | undefined = {
        enableHighAccuracy: true,
        timeout: Infinity,
        maximumAge: 0,
    };
    if (navigator.geolocation) {
        navigator.geolocation.watchPosition(success, error, options);
    } else {
        alert('위치정보 사용 불가능');
    }
}
