import { viteConfig } from './viteConfig';
import jwtAxios from '../utils/jwtUtil';
import { RsvInputType, TogetherRsvInputType } from '../interfaces/Reservation';

const host = `${viteConfig.VITE_BASE_URL}/api/reservations`;

// 전체 예약 목록 가져오기
export const getRsvList = async () => {
	const res = await jwtAxios.get(`${host}/`);
	return res.data;
};

// 내 예약 목록 가져오기
export const getMyRsvList = async () => {
	const res = await jwtAxios.get(`${host}/my`);
	return res.data;
};

// 예약 추가 (NEW)
export const addRsv = async (rsvObj: RsvInputType) => {
	const res = await jwtAxios.post(`${host}/`, rsvObj);
	return res.data;
};

// 예약 추가 (합승)
export const addTogetherRsv = async (rsvId: number, togetherRsvObj: TogetherRsvInputType) => {
	const res = await jwtAxios.post(`${host}/${rsvId}`, togetherRsvObj);
	return res.data;
};

// 예약 삭제
export const deleteRsv = async (rsvId: number, placeId: number) => {
	const res = await jwtAxios.delete(`${host}/${rsvId}/${placeId}`);
	return res.data;
};
