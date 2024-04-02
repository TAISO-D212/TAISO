import { create } from 'zustand';
import { persist, createJSONStorage } from 'zustand/middleware';

interface TogetherRsvStoreState {
	bookmarkId?: number | null;
	setBookmarkId?: (value: number | null) => void;
	latitude?: number | null;
	setLatitude?: (value: number | null) => void;
	longitude?: number | null;
	setLongitude?: (value: number | null) => void;
	address?: string | null;
	setAddress?: (value: string | null) => void;
	cnt: number | null;
	setCnt: (value: number | null) => void;

}

const TogetherRsvStore = create(
	persist<TogetherRsvStoreState>(
		(set) => ({
			bookmarkId: null,
			setBookmarkId: (value) => set({ bookmarkId: value }),
			latitude: null,
			setLatitude: (value) => set({ latitude: value }),
			longitude: null,
			setLongitude: (value) => set({ longitude: value }),
			address: null,
			setAddress: (value) => set({ address: value }),
			cnt: 1,
			setCnt: (value) => set({ cnt: value }),
		}),
		{
			name: 'TogetherRsv',
			storage: createJSONStorage(() => localStorage),
		},
	),
);

export default TogetherRsvStore;
