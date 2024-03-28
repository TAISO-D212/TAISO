export interface RsvInputType {
	startPlaceId?: number;
	startLatitude?: number;
	startLongitude?: number;
	startAddress?: string;
	endPlaceId?: number;
	endLatitude?: number;
	endLongitude?: number;
	endAddress?: string;
	time: string;
	cnt: number;
}

// 합승 시에는 출발지만 정보 기입
export interface TogetherRsvInputType {
	placeId?: number;
	latitude?: number;
	longitude?: number;
	address?: string;
	cnt: number;
}
