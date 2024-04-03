import React from 'react';
import { useState, useEffect } from 'react';
import { useNavigate } from 'react-router-dom';
import Logo192 from '../../assets/icon/icon_192.png';
import Logo72 from '../../assets/icon/icon_72.png';
import LogoImage from '../../assets/image/Loding_Image.png';
import useCustomLogin from '../../hooks/useCustomLogin';
// import { SignUpPage } from '../signup/SignUpPage';

export const LoginPage = () => {
	const navigate = useNavigate();

	const { isLogin } = useCustomLogin();

	const [loading, setLoading] = useState(true);
	// const [modalIsOpen, setModalIsOpen] = useState(false);

	const { doLogin } = useCustomLogin();

	useEffect(() => {
		if (isLogin) {
			navigate('/main');
		} else {
			setTimeout(() => {
				setLoading(false);
			}, 2000);
		}
	}, []);

	// const handleSignUpModal = () => {
	// 	setModalIsOpen((modalIsOpen) => !modalIsOpen);
	// };

	interface LoginCredentials {
		email: string;
		pw: string;
	}

	const [loginParams, setLoginParams] = React.useState<LoginCredentials>({ email: '', pw: '' });

	const handleChange = (e: React.ChangeEvent<HTMLInputElement>) => {
		if (e.target.name in loginParams) {
			loginParams[e.target.name as keyof LoginCredentials] = e.target.value;
		}
		setLoginParams({ ...loginParams });
	};

	const onClickLogInHandler = () => {
		// dispatch(login(loginParam))

		// 차후 수정 예정
		doLogin(loginParams)
			.then((data) => {
				console.log(data);

				if (data.error) {
					alert('이메일과 패스워드를 다시 확인하세요');
				} else {
					alert('로그인 성공');
					// 뒤로가기 막기
					navigate('/main', { replace: true });
				}
			})
			.then(() => {});
	};

	const onClickSignUpHandler = () => {
		navigate('/signup');
	};

	if (loading)
		return (
			<>
				<div className='w-[100%] h-[100%] flex flex-col justify-center items-center animate-fadeIn'>
					<img src={Logo192} alt='TAISO LOGO 192SIZE' />
					<div className="mt-[15px] font-['Pretendard-Bold'] text-[60px]">TAISO</div>
					<div className='mt-[15px] w-[90%] h-100'>
						<img src={LogoImage} alt='TAISO SERVICE IMAGE' />
					</div>
				</div>
			</>
		);
	return (
		<>
			<div className='w-[100%] h-[100%] flex flex-col justify-center items-center'>
				<div className='w-[90%] h-24 flex justify-evenly items-center my-[5%]'>
					<div>
						<img src={Logo72} alt='TAISO LOGO 72SIZE' />
					</div>
					<div className="font-['Pretendard-Bold'] text-[50px]">타이소</div>
				</div>
				<div className='h-24 flex flex-col justify-center items-center my-[5%]'>
					<div className="font-['Pretendard-Bold'] text-[36px] my-[5%]">Login</div>
					<div className='text-disabled'>아이디 비밀번호를 입력해 주세요.</div>
				</div>
				<div className='w-[90%] h-[50%] flex flex-col justify-center items-center'>
					<input
						className='h-14 w-[90%] placeholder:text-gray block border border-gray rounded-full focus:outline-none focus:border-sky-500 my-[5%] px-[5%]'
						name='email'
						type={'text'}
						value={loginParams.email}
						onChange={handleChange}
						placeholder='이메일'></input>
					<input
						className='h-14 w-[90%] laceholder:text-gray block border border-gray rounded-full focus:outline-none focus:border-sky-500 my-[5%] px-[5%]'
						name='pw'
						type={'password'}
						value={loginParams.pw}
						onChange={handleChange}
						placeholder='비밀번호'></input>
					<div
						className="h-14 w-[90%] bg-[#3422F2] rounded-full text-white font-['Pretendard-Bold'] text-[20px] my-[5%] px-[5%] flex justify-center items-center hover:cursor-pointer"
						onClick={onClickLogInHandler}>
						로그인
					</div>
					<div
						className="w-[100%] font-['Pretendard-Bold'] mt-[10%] px-[5%] text-center"
						onClick={onClickSignUpHandler}>
						처음 이용하시는 유저신가요?
						<span>&nbsp;&nbsp;&nbsp;&nbsp;</span>
						{/* <span className='text-blue hover:underline' onClick={handleSignUpModal}> */}
						<span className='text-[#3422F2] hover:underline'>회원가입</span>
					</div>
				</div>
			</div>
			{/* {modalIsOpen && <SignUpPage setModalIsOpen={setModalIsOpen}/>} */}
		</>
	);
};
