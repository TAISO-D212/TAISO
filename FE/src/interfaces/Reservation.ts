export interface RsvInputType {
	startBookmarkId?: number;
	startLatitude?: number;
	startLongitude?: number;
	startAddress?: string;
	endBookmarkId?: number;
	endLatitude?: number;
	endLongitude?: number;
	endAddress?: string;
	time: string;
	cnt: number;
}

// 합승 시에는 출발지만 정보 기입 (도착지 x)
export interface TogetherRsvInputType {
	bookmarkId?: number;
	latitude?: number;
	longitude?: number;
	address?: string;
	cnt: number;
}

export interface RsvType {
	rsvId: number;
	placeId: number;
	latitude: number;
	longitude: number;
	address: string;
	time: string;
	arrivalTime?: string;
	stopCnt: number;
	cnt: number;
}

export interface MyRsvType {
	rsvId: number;
	startPlaceId: number;
	startLatitude: number;
	startLongitude: number;
	startAddress: string;
	endPlaceId: number;
	endLatitude: number;
	endLongitude: number;
	endAddress: string;
	time: string;
	arrivalTime?: string;
	cnt: number;
}
