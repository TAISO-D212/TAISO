import axios from 'axios';
import { viteConfig } from './viteConfig';
import { SignUpInputType } from '../interfaces/Member';
import jwtAxios from '../utils/jwtUtil';

export type TLoginParam = {
	email: string;
	pw: string;
};

const host = `${viteConfig.VITE_BASE_URL}/api/members`;

export const loginPost = async (loginParam: TLoginParam) => {
	// 일반 post 방식으로 데이터를 보내므로
	const header = { headers: { 'Content-Type': 'x-www-form-urlencoded' } };

	const form = new FormData();
	form.append('username', loginParam.email);
	form.append('password', loginParam.pw);

	const res = await axios.post(`${host}/login`, form, header);

	return res.data;
};

export const checkEmail = async (email: string) => {
	const header = { headers: { 'Content-Type': 'application/json' } };

	const res = await axios.get(`${host}/ck/${email}`, header);

	return res.data;
};

export const join = async (signUpObj: SignUpInputType) => {
	const header = { headers: { 'Content-Type': 'application/json' } };

	const res = await axios.post(`${host}/join`, signUpObj, header);

	return res.data;
};

export const deleteMember = async () => {
	const res = await jwtAxios.delete(`${host}/`);
	return res.data;
};
