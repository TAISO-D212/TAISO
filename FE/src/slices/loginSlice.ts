import { PayloadAction, createAsyncThunk, createSlice } from '@reduxjs/toolkit';
import { loginPost, TLoginParam } from '../apis/memberApi';

const initState = {
	email: '',
};


export const loginPostAsync = createAsyncThunk("loginPostAsync", (param:TLoginParam) =>
  loginPost(param)
);

// reducer는 금고를 어떻게 할 것이냐
// reducer 안에 action(입력값)이 있고 얘가 리턴을 해주는 것이 앞으로 계속 유지해야 되는 데이터이다.

//dispatch는 뿌리는 애, selector는 영향을 받는 애

const loginSlice = createSlice({
	name: 'loginSlice',
	initialState: initState,
	reducers: {
		login: (state, action:PayloadAction<TLoginParam>) => {
			console.log('login..........', action);
      console.log(action.payload)
      return {email : action.payload.email}
		},
		logout: () => {
			console.log('logout......');
      return{...initState}
		},
	},
  extraReducers: (builder) => {
    // fulfilled : 완료가 되었다, pending: 처리 중, rejected: 문제 발생
    builder
      .addCase(loginPostAsync.fulfilled, (state, action) => {
        console.log("fulfilled");

        const payload = action.payload

        // if (!payload.error){
        //   setCookie("member", JSON.stringify(payload), 1)
        // }

        return payload
      })
      .addCase(loginPostAsync.pending, () => {
        console.log("pending");
      })
      .addCase(loginPostAsync.rejected, () => {
        console.log("rejected");
      });
  },
});

// 이부분 잘 보고 써먹으세요!! (함수 호출 쉽게 하려고)
export const { login, logout } = loginSlice.actions;

export default loginSlice.reducer;



// 초기화면 부분에서 쓰일듯...
//(cf) 다른곳에서 상태 보려면
// const loginState = useSelector((state) => state.loginSlice);
