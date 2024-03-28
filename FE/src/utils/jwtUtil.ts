import axios, { AxiosError, AxiosInstance, AxiosResponse, InternalAxiosRequestConfig } from 'axios';

import { getCookie, setCookie } from './cookieUtil';
import { viteConfig } from '../apis/viteConfig';

const jwtAxios: AxiosInstance = axios.create();

const refreshJWT = async (accessToken: string, refreshToken: string) => {
	const host = viteConfig.VITE_BASE_URL;

	const header = { headers: { Authorization: `Bearer ${accessToken}` } };

	const res = await axios.get(`${host}/api/members/refresh?refreshToken=${refreshToken}`, header);

	//console.log("----------------------")
	//console.log(res.data)

	return res.data;
};

//before request
const beforeReq = (
	config: InternalAxiosRequestConfig<any>,
): InternalAxiosRequestConfig<any> | Promise<any> => {
	console.log('before request.............');

  // 쿠키로 부터 정보를 받고 그 값을 통해 헤더 처리

	const memberInfo = getCookie('member');

	if (!memberInfo) {
		console.log('Member NOT FOUND');
		return Promise.reject({ response: { data: { error: 'REQUIRE_LOGIN' } } });
	}

	const { accessToken } = memberInfo;

	// Authorization 헤더 처리
	config.headers.Authorization = `Bearer ${accessToken}`;

	return config;
};

//fail request
const requestFail = (err: AxiosError | Error): Promise<AxiosError> => {
	console.log('request error............');

	return Promise.reject(err);
};

//before return response
const beforeRes = async (res: AxiosResponse): Promise<any> => {
	console.log('before return response...........');

	const data = res.data;

	if (data && data.error === 'ERROR_ACCESS_TOKEN') {
		const memberCookieValue = getCookie('member');

		const result = await refreshJWT(memberCookieValue.accessToken, memberCookieValue.refreshToken);
		console.log('refreshJWT RESULT', result);

    // 새로운 accessToken, accessToken

		memberCookieValue.accessToken = result.accessToken;
		memberCookieValue.refreshToken = result.refreshToken;

		setCookie('member', JSON.stringify(memberCookieValue), 10);

		//원래의 호출(정상적인 토큰들로)
		const originalRequest = res.config;

		originalRequest.headers.Authorization = `Bearer ${result.accessToken}`;

		return await axios(originalRequest);
	}

	return res;
};

//fail response
const responseFail = (err: AxiosError | Error): Promise<Error> => {
	console.log('response fail error.............');

	return Promise.reject(err);
};

jwtAxios.interceptors.request.use(beforeReq, requestFail);

jwtAxios.interceptors.response.use(beforeRes, responseFail);

export default jwtAxios;
