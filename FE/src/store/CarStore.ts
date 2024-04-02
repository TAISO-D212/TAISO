import { create } from 'zustand';
import { persist } from 'zustand/middleware';

interface ILocation {
	La: number;
	Ma: number;
}

interface CarStoreState {
	carLocation: number[] | [];
	setCarLocation: (latlng: number[]) => void;
	carLocationList: ILocation[] | [];
	setCarLocationList: (carArea: ILocation) => void;
	resetCarLocationList: () => void;
}

export const CarStore = create(
	persist<CarStoreState>(
		(set) => ({
			carLocation: [],
			setCarLocation: (latlng) => set({ carLocation: latlng }),
			carLocationList: [],
			setCarLocationList: (carArea) =>
				set((state) => ({ carLocationList: [...state.carLocationList, carArea] })),
			resetCarLocationList: () => set({ carLocationList: [] }),
		}),
		{
			name: 'Car',
			getStorage: () => localStorage,
		},
	),
);
