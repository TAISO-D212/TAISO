import { BottomNav } from '../../components/BottomNav';
import axios from 'axios';
import { ReservationListElement } from './components/ReservationListElement.tsx';
import { useEffect, useState } from 'react';
import { useNavigate } from 'react-router';
import { BackButton } from '../../components/BackButton.tsx';
import ReservationButton from '../../assets/image/reservation_button.png';

export const Reservation = () => {
	const [requestArray, setRequestArray] = useState([]);
	const navigate = useNavigate();

	// useEffect(() => {
	// 	axios({
	// 		headers: {
	// 			Barear: 'token',
	// 		},
	// 		method: 'get',
	// 		url: '주소',
	// 		responseType: 'json',
	// 	}).then((response) => {
	// 		setRequestArray(response.data);
	// 	});
	// }, []);

	const goNewReservation = () => {
		navigate('/reservation/new');
	};

	return (
		<>
			<BackButton />
			<div className='fixed bottom-[90px] w-[100%] h-[80%] flex flex-col justify-center items-center'>
				<div className='fixed flex top-[10%] w-[90%] h-[8%] px-5 shadow-md border border-[#d9d9d9] rounded-full justify-between items-center'>
					원하는 목적지가 없으신가요?
					<div onClick={goNewReservation} className='w-[45px] h-[45px]'>
						<img src={ReservationButton} alt='신규 예약하기' />
					</div>
				</div>
				<div className='w-[100%] flex justify-center'>
					<div className='fixed w-[100%] top-[20%] flex font-[#a9a9a9] justify-center items-center text-center'>
						<div className="font-['Pretendard-Bold'] text-[26px] my-[2%]">예약목록</div>
					</div>
				</div>
				{requestArray.length !== 0 ? (
					requestArray.map((e) => {
						return <ReservationListElement reservationContent={e} />;
					})
				) : (
					<div className='fixed bottom-[90px] w-[100%] h-[60%] flex flex-col justify-center items-center'>
						예약 가능한 내역이 없습니다.
					</div>
				)}
			</div>
			<BottomNav />
		</>
	);
};
