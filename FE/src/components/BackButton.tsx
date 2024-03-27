import { useNavigate } from 'react-router';
import Back from '../assets/image/back.png';

export const BackButton = () => {
	const navigate = useNavigate();

	const onClickBtn = () => {
		navigate(-1);
	};

	return (
		<>
			<div className='fixed w-[40px] h-[40px] top-[3%] left-[3%] z-10' onClick={onClickBtn}>
				<img src={Back} alt='뒤로가기' />
			</div>
		</>
	);
};
