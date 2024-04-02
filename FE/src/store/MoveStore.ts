import { create } from 'zustand';
import { persist } from 'zustand/middleware';

interface ILocation {
	La: number;
	Ma: number;
}

interface MoveStoreState {
	location: number[] | [];
	setLocation: (latlng: number[]) => void;
	locationList: ILocation[] | [];
	setLocationList: (area: ILocation) => void;
	resetLocationList: () => void;
	totalTime: string;
	setTotalTime: (time: string) => void;
	totalDistance: number;
	setTotalDistance: (dis: number) => void;
	time: number;
	setTime: () => void;
	resetTime: () => void;
}

export const MoveStore = create(
	persist<MoveStoreState>(
		(set) => ({
			location: [],
			setLocation: (latlng) => set({ location: latlng }),
			locationList: [],
			setLocationList: (area) => set((state) => ({ locationList: [...state.locationList, area] })),
			resetLocationList: () => set({ locationList: [] }),
			totalTime: '00:00:00',
			setTotalTime: (time) => set({ totalTime: time }),
			totalDistance: 0,
			setTotalDistance: (dis) => set({ totalDistance: dis }),
			time: 0,
			setTime: () => set((state) => ({ time: state.time + 1 })),
			resetTime: () => set({ time: 0 }),
		}),
		{
			name: 'User',
			getStorage: () => localStorage,
		},
	),
);
