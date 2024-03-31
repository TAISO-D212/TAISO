import { create } from 'zustand';
import { persist, createJSONStorage } from 'zustand/middleware';

interface NewReservationStoreState {
	startPlaceId?: number | null;
	setStartPlaceId?: (value: number | null) => void;
	startLatitude?: number | null;
	setStartLatitude?: (value: number | null) => void;
	startLongitude?: number | null;
	setStartLongitude?: (value: number | null) => void;
	startAddress?: string | null;
	setStartAddress?: (value: string | null) => void;
	endPlaceId?: number | null;
	setEndPlaceId?: (value: number | null) => void;
	endLatitude?: number | null;
	setEndLatitude?: (value: number | null) => void;
	endLongitude?: number | null;
	setEndLongitude?: (value: number | null) => void;
	endAddress?: string | null;
	setEndAddress?: (value: string | null) => void;
	time: string | null;
	setTime: (value: string | null) => void;
	cnt: number | null;
	setCnt: (value: number | null) => void;

	// description: string;
	// setDescription: (value: string) => void;
	// title: string;
	// setTitle: (value: string) => void;
	// distance: number;
	// setDistance: (value: number) => void;
	// destination: string | null;
	// setDestination: (value: string | null) => void;
	// totalPeople: number;
	// setTotalPeople: (value: number) => void;
	// selectedTime: string;
	// setSelectedTime: (value: string) => void;
	// postId: number;
	// setPostId: (value: number) => void;
}

const NewReservationStore = create(
	persist<NewReservationStoreState>(
		(set) => ({
			startPlaceId: null,
			setStartPlaceId: (value) => set({ startPlaceId: value }),
			startLatitude: null,
			setStartLatitude: (value) => set({ startLatitude: value }),
			startLongitude: null,
			setStartLongitude: (value) => set({ startLongitude: value }),
			startAddress: null,
			setStartAddress: (value) => set({ startAddress: value }),
			endPlaceId: null,
			setEndPlaceId: (value) => set({ endPlaceId: value }),
			endLatitude: null,
			setEndLatitude: (value) => set({ endLatitude: value }),
			endLongitude: null,
			setEndLongitude: (value) => set({ endLongitude: value }),
			endAddress: null,
			setEndAddress: (value) => set({ endAddress: value }),
			time: null,
			setTime: (value) => set({ time: value }),
			cnt: null,
			setCnt: (value) => set({ cnt: value }),
			// description: '',
			// title: '',
			// distance: 0,
			// destination: null,
			// totalPeople: 1,
			// selectedTime: '',
			// postId: 0,
			// setDescription: (value) => set({ description: value }),
			// setTitle: (value) => set({ title: value }),
			// setDistance: (value) => set({ distance: value }),
			// setDestination: (value) => set({ destination: value }),
			// setTotalPeople: (value: number) => set({ totalPeople: value }),
			// setSelectedTime: (value) => set({ selectedTime: value }),
			// setPostId: (value: number) => set({ postId: value }),
		}),
		{
			name: 'NewReservation',
			storage: createJSONStorage(() => localStorage),
		},
	),
);

export default NewReservationStore;
