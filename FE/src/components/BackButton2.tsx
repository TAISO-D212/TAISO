import { useNavigate } from 'react-router';
import Back from '../assets/image/back.png';

export const BackButton2 = () => {
	const navigate = useNavigate();

	const onClickBtn = () => {
		navigate(-1);
	};

	return (
		<>
			<div
				className='w-[40px] h-[40px] z-10 animate-fadeIn'
				onClick={onClickBtn}>
				<img src={Back} alt='뒤로가기' />
			</div>
		</>
	);
};
