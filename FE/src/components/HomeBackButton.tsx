import { useNavigate } from 'react-router';
import Back from '../assets/image/back.png';

export const HomeBackButton = () => {
	const navigate = useNavigate();

	const onClickBtn = () => {
		navigate('/main', { replace: true });
	};

	return (
		<>
			<div
				className='fixed w-[40px] h-[40px] top-[3.2%] left-[3%] z-10 animate-fadeIn'
				onClick={onClickBtn}>
				<img src={Back} alt='뒤로가기' />
			</div>
		</>
	);
};
