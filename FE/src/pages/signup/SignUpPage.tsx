import { useState, FormEvent } from 'react';
import {
	Avatar,
	Button,
	CssBaseline,
	TextField,
	FormControl,
	FormHelperText,
	Grid,
	Box,
	Typography,
	Container,
} from '@mui/material/';
import CheckCircleIcon from '@mui/icons-material/CheckCircle';
import { createTheme, ThemeProvider } from '@mui/material/styles';
import styled from 'styled-components';
import { BackButton } from '../../components/BackButton';
import { SignUpInputType } from '../../interfaces/Member';
import { checkEmail, memberJoin } from '../../apis/memberApi';
import useCustomLogin from '../../hooks/useCustomLogin';

// 이메일, 비밀번호, 이름을 나타내는 인터페이스 정의
interface FormData {
	email: string;
	pwd: string;
	name: string;
	rePassword: string;
}

// mui의 css 우선순위가 높기때문에 important를 설정
const FormHelperTexts = styled(FormHelperText)`
	width: 100%;
	padding-left: 16px;
	font-weight: 700 !important;
	color: #d32f2f !important;
`;

const Boxs = styled(Box)`
	padding-bottom: 40px !important;
`;

export const SignUpPage = () => {
	const theme = createTheme();
	const [emailError, setEmailError] = useState<string>('');
	const [passwordState, setPasswordState] = useState<string>('');
	const [passwordError, setPasswordError] = useState<string>('');
	const [nameError, setNameError] = useState<string>('');
	const [registerError, setRegisterError] = useState<string>('');
	const [color, setColor] = useState<string>('info');

	const { moveToLogin } = useCustomLogin();

	
	const [email, setEmail] = useState<string>('');

	// 이메일 입력시 변화
	const handleEmailChange = (e:React.ChangeEvent<HTMLInputElement>) => {
		setEmail(e.target.value);
	};

	// 이메일 중복 확인
	const onHandleCheckEmail = () => {
		const result = checkEmail(email);
		console.log(result);
	};

	// 회원 가입 로직
	const onhandlePost = async (data: FormData) => {
		const { email, name, pwd } = data;
		const postData: SignUpInputType = { email, name, pwd };
		try {
			const response = memberJoin(postData);
			console.log(response, '성공');
			// 다시 로그인 하세요!!
			moveToLogin();
		} catch (err) {
			console.log(err);
			setRegisterError('회원가입에 실패하였습니다. 다시한번 확인해 주세요.');
		}
	};

	const handleSubmit = (e: FormEvent<HTMLFormElement>) => {
		e.preventDefault();

		const data = new FormData(e.currentTarget);
		const joinData: FormData = {
			email: data.get('email') as string,
			name: data.get('name') as string,
			pwd: data.get('pwd') as string,
			rePassword: data.get('rePassword') as string,
		};
		const { email, name, pwd, rePassword } = joinData;

		// 이메일 유효성 체크
		const emailRegex =
			/([\w-.]+)@((\[[0-9]{1,3}\.[0-9]{1,3}\.[0-9]{1,3}\.)|(([\w-]+\.)+))([a-zA-Z]{2,4}|[0-9]{1,3})(\]?)$/;
		if (!emailRegex.test(email)) setEmailError('올바른 이메일 형식이 아닙니다.');
		else setEmailError('');

		// 비밀번호 유효성 체크
		const passwordRegex = /^(?=.*[a-zA-Z])(?=.*\d).{8,25}$/;
		if (!passwordRegex.test(pwd)) setPasswordState('숫자와 영문자 조합으로 8자리 이상 입력해주세요!');
		else setPasswordState('');

		// 비밀번호 같은지 체크
		if (pwd !== rePassword) setPasswordError('비밀번호가 일치하지 않습니다.');
		else setPasswordError('');

		// 이름 유효성 검사
		const nameRegex = /^[가-힣a-zA-Z]+$/;
		if (!nameRegex.test(name) || name.length < 1) setNameError('올바른 이름을 입력해주세요.');
		else setNameError('');

		if (
			email &&
			pwd &&
			name &&
			rePassword &&
			emailRegex.test(email) &&
			passwordRegex.test(pwd) &&
			pwd === rePassword &&
			nameRegex.test(name)
		) {
			onhandlePost(joinData);
		}
	};

	return (
		<div className='w-[100%] h-[100] flex justify-center items-center animate-fadeIn'>
			<BackButton />
			<ThemeProvider theme={theme}>
				<Container component='main' maxWidth='xs'>
					<CssBaseline />
					<Box
						sx={{
							marginTop: 8,
							display: 'flex',
							flexDirection: 'column',
							alignItems: 'center',
						}}>
						<Avatar sx={{ width: 100, height: 100, m: 1, bgcolor: '#c4b5fc', marginBottom: '10%' }} />
						<Typography component='h1' variant='h5' sx={{ marginBottom: '10%' }}>
							회원가입
						</Typography>
						<form className='pb-[40px]' noValidate onSubmit={handleSubmit}>
							<FormControl component='fieldset' variant='standard'>
								<Grid container spacing={2}>
									<Grid item xs={12}>
										<div className='flex'>
											<TextField
												required
												autoFocus
												sx={{ width: '80%' }}
												type='email'
												id='email'
												name='email'
												label='이메일 주소'
												error={emailError !== '' || false}
												onChange={handleEmailChange}
											/>
											<Button
												variant='contained'
												color={color}
												// TODO : 이메일 체크 후 버튼 색상 변경 -> 변경색은 success , error
												sx={{ width: '10%', height: 55, display: 'inline' }}
												onClick={onHandleCheckEmail}>
												<CheckCircleIcon />
											</Button>
										</div>
									</Grid>
									<FormHelperTexts>{emailError}</FormHelperTexts>
									<Grid item xs={12}>
										<TextField
											required
											fullWidth
											type='password'
											id='pwd'
											name='pwd'
											label='비밀번호'
											error={passwordState !== '' || false}
										/>
									</Grid>
									<FormHelperTexts>{passwordState}</FormHelperTexts>
									<Grid item xs={12}>
										<TextField
											required
											fullWidth
											type='password'
											id='rePassword'
											name='rePassword'
											label='비밀번호 재입력'
											error={passwordError !== '' || false}
										/>
									</Grid>
									<FormHelperTexts>{passwordError}</FormHelperTexts>
									<Grid item xs={12}>
										<TextField
											required
											fullWidth
											id='name'
											name='name'
											label='이름'
											error={nameError !== '' || false}
										/>
									</Grid>
									<FormHelperTexts>{nameError}</FormHelperTexts>
								</Grid>
								<Button
									type='submit'
									fullWidth
									variant='contained'
									sx={{ mt: 5, mb: 2, bgcolor: '#c4b5fc' }}
									size='large'>
									<div className='font-["Pretendard-Bold"] text-[20px]'>회원가입</div>
								</Button>
							</FormControl>
							<FormHelperTexts>{registerError}</FormHelperTexts>
						</form>
					</Box>
				</Container>
			</ThemeProvider>
		</div>
	);
};

// export const SignUpPage = () => {
// 	return (
// 		<>
// 			<div className='w-[90%] h-[90%] flex flex-col justify-center items-center animate-fadeIn'>
// 				<div className='modal-header'>
// 					<h5 className='modal-title'>회원가입</h5>
// 				</div>
// 				<div className='modal-content'>
// 					<div className='h-10 w-[90%] flex justify-end items-center my-[5%]'>
// 						<input
// 							className='h-10 w-[65%] placeholder:text-gray block border border-gray rounded-full focus:outline-none focus:border-sky-500 my-[5%] px-[5%] mr-[5%]'
// 							placeholder='이메일'></input>
// 						<div className='h-10 w-[30%] bg-blue rounded-full text-white my-[5%] px-[5%] flex justify-center items-center'>
// 							중복확인
// 						</div>
// 					</div>
// 					<input
// 						className='h-10 w-[90%] laceholder:text-gray block border border-gray rounded-full focus:outline-none focus:border-sky-500 my-[5%] px-[5%]'
// 						placeholder='비밀번호'></input>
// 					<input
// 						className='h-10 w-[90%] laceholder:text-gray block border border-gray rounded-full focus:outline-none focus:border-sky-500 my-[5%] px-[5%]'
// 						placeholder='비밀번호 확인'></input>
// 				</div>
// 				<div className='modal-footer'>
// 					<button
// 						className='modal-button'
// 						// onClick={() => }
// 					>
// 						회원가입
// 					</button>
// 				</div>
// 			</div>
// 		</>
// 	);
// };
