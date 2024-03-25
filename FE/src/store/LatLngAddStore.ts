import { create } from 'zustand';

interface LatLngAddStore {
    currentLat: number;
    currentLng: number;
    currentAdd?: string;
    setLatLngAdd: (currentLat: number, currentLng: number, currentAdd?: string) => void;
    removeAll: () => void;
}

const LatLngAddStore = create<LatLngAddStore>((set) => ({
    currentLat: 0,
    currentLng: 0,
    currentAdd: '',
    setLatLngAdd: (Lat, Lng, currentAdd) => set(() => ({ currentLat: Lat, currentLng: Lng, currentAdd: currentAdd })),
    removeAll: () => set(() => ({ currentLat: 0, currentLng: 0, currentAdd: '' })),
}));

export default LatLngAddStore;
