import Home from '../assets/nav/Home.png';
import Reservation from '../assets/nav/Reservation.png';
import History from '../assets/nav/History.png';
import Favorite from '../assets/nav/Favorite.png';
import Profile from '../assets/nav/Profile.png';
import { useNavigate } from 'react-router-dom';

export const BottomNav = () => {
	const navigate = useNavigate();

	const goHome = () => {
		navigate('/main');
	};
	const goReservation = () => {
		navigate('/reservation');
	};
	const goProfile = () => {
		navigate('/profile');
	};
	const goHistory = () => {
		navigate('/history');
	};

	return (
		<div className='fixed bottom-0 backdrop-blur-sm rounded-[24px] z-10 w-[95%] h-[65px] flex justify-evenly items-center animate-fadeIn'>
			<div className='w-[42px] z-10 flex flex-col justify-center items-center' onClick={goHome}>
				<img src={Home} alt='HOME_TAB' />
				<div className='text-[12px] mt-[5px]'>홈</div>
			</div>
			<div className='w-[42px] flex flex-col justify-center items-center' onClick={goReservation}>
				<img src={Reservation} alt='RESERVATION_TAB' />
				<div className='text-[12px] mt-[5px]'>예약</div>
			</div>
			<div className='w-[42px] flex flex-col justify-center items-center' onClick={goHistory}>
				<img src={History} alt='HISTORY_TAB' />
				<div className='text-[12px] mt-[5px]'>이용내역</div>
			</div>
			<div className='w-[42px] flex flex-col justify-center items-center'>
				<img src={Favorite} alt='FAVORITE_TAB' />
				<div className='text-[12px] mt-[5px]'>즐겨찾기</div>
			</div>
			<div className='w-[42px] flex flex-col justify-center items-center' onClick={goProfile}>
				<img src={Profile} alt='PROFILE_TAB' />
				<div className='text-[12px] mt-[5px]'>내 정보</div>
			</div>
		</div>
	);
};
