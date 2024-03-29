import { useDispatch, useSelector } from 'react-redux';
import { useNavigate } from 'react-router-dom';
import { loginPostAsync, logout } from '../slices/loginSlice';
import { AppDispatch, RootState } from '../store/LoginStore';
import { TLoginParam } from '../apis/memberApi';
import { Member } from '../interfaces/Member';

export interface TCusotmLogin {
	loginState: Member;
	isLogin: boolean;
	doLogin: (loginParam: TLoginParam) => Promise<any>;
	doLogout: () => void;
	moveToPath: (path: string) => void;
	moveToLogin: () => void;
}

const useCustomLogin = (): TCusotmLogin => {
	const navigate = useNavigate();

	const dispatch = useDispatch<AppDispatch>();

	const loginState = useSelector((state: RootState) => state.loginSlice);

	const isLogin = loginState.email ? true : false; //----------로그인 여부

	const doLogin = async (loginParam: TLoginParam) => {
		//----------로그인 함수

		const action = await dispatch(loginPostAsync(loginParam));

		return action.payload;
	};

	const doLogout = () => {
		//---------------로그아웃 함수

		dispatch(logout(null));
	};

	const moveToPath = (path: string) => {
		//----------------페이지 이동
		navigate({ pathname: path }, { replace: true });
	};

	const moveToLogin = () => {
		//----------------------로그인 페이지로 이동
		navigate('/', { replace: true });
	};

	return { loginState, isLogin, doLogin, doLogout, moveToPath, moveToLogin };
};

export default useCustomLogin;
