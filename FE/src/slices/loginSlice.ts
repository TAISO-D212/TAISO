import { PayloadAction, createAsyncThunk, createSlice } from '@reduxjs/toolkit';
import { loginPost, TLoginParam } from '../apis/memberApi';
import { getCookie, removeCookie, setCookie } from '../utils/cookieUtil';
import { Member } from '../interfaces/Member';

const loadMemberCookie = (): Member => {
	//쿠키에서 로그인 정보 로딩

	const memberInfo = getCookie('member');

	//닉네임 처리
	// if (memberInfo && memberInfo.name) {
	// 	memberInfo.name = decodeURIComponent(memberInfo.name);
	// }

	return memberInfo;
};

export const loginPostAsync = createAsyncThunk('loginPostAsync', (param: TLoginParam) =>
	loginPost(param),
);

// reducer는 금고를 어떻게 할 것이냐
// reducer 안에 action(입력값)이 있고 얘가 리턴을 해주는 것이 앞으로 계속 유지해야 되는 데이터이다.

//dispatch는 뿌리는 애, selector는 영향을 받는 애

const loginSlice = createSlice({
	name: 'loginSlice',
	initialState: loadMemberCookie() || { email: '' },
	reducers: {
		login: (state, action: PayloadAction<TLoginParam>) => {
			console.log('login..........', action);
			console.log(action.payload);
			return { email: action.payload.email };
		},
		logout: () => {
			console.log('logout......');
			removeCookie('member');
      
			return { email: '' };
		},
	},
	extraReducers: (builder) => {
		// fulfilled : 완료가 되었다, pending: 처리 중, rejected: 문제 발생
		builder
			.addCase(loginPostAsync.fulfilled, (state, action) => {
				console.log('fulfilled');

				const payload = action.payload;

				// 정상적인 로그인 시에만 저장
				if (!payload.error) {
					setCookie('member', JSON.stringify(payload), 10);
				}

				return payload;
			})
			.addCase(loginPostAsync.pending, () => {
				console.log('pending');
			})
			.addCase(loginPostAsync.rejected, () => {
				console.log('rejected');
			});
	},
});

// 이부분 잘 보고 써먹으세요!! (함수 호출 쉽게 하려고)
export const { login, logout } = loginSlice.actions;

export default loginSlice.reducer;

// 초기화면 부분에서 쓰일듯...
//(cf) 다른곳에서 상태 보려면
// const loginState = useSelector((state) => state.loginSlice);
