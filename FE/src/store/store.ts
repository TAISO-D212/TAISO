import create from 'zustand';

interface Store {
  currentLat: number;
  currentLng: number;
  setLatLng: (currentLat: number, currentLng: number) => void;
  removeAll: () => void;
}

export const LatLngStore = create<Store>((set) => ({
  currentLat: 0,
  currentLng: 0,
  setLatLng: (Lat, Lng) => set(() => ({ currentLat: Lat, currentLng: Lng })),
  removeAll: () => set(() => ({ currentLat: 0, currentLng: 0 })),
}));