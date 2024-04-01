import axios from 'axios';

// 이동 시작시 요청
export const postStartMove = async (lat: number, lon: number, token: string | null) => {
	try {
		const response = await axios.post(
			`${import.meta.env.VITE_API_BASE_URL}/혹시 이런게 있나?`,
			{
				time: '00:00:00',
				distance: 0,
				latitude: lat,
				longitude: lon,
			},
			{
				headers: {
					Authorization: token,
				},
			},
		);
		return response.data;
	} catch (error) {
		console.error('Move Start fetch Failed:', error);
	}
};
interface Location {
	La: number;
	Ln: number;
}

interface IMoveEnd {
	time: string;
	distance: number;
	spotLists: Location[] | [];
	token: string | null;
}

// 이동 종료시 요청
export const postEndMove = async ({ time, distance, spotLists, token }: IMoveEnd) => {
	try {
		const response = await axios.put(
			`${import.meta.env.VITE_API_BASE_URL}/뭐를 보내야 합니까?`,
			{
				time,
				distance,
				spotLists,
			},
			{
				headers: {
					Authorization: token,
				},
			},
		);
		console.log(response);
	} catch (error) {
		console.error('Move End fetch Failed:', error);
		throw error;
	}
};
