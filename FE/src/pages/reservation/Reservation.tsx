import { BottomNav } from '../../components/BottomNav';
import axios from 'axios';
import { ReservationListElement } from './components/ReservationListElement.tsx';
import { useEffect, useState } from 'react';
import { useNavigate } from 'react-router';
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
			<div className='fixed w-[100%] bottom-[90px] flex flex-col items-center'>
				<div className='fixed flex top-[5%] w-[90%] h-[8%] px-3 shadow-md border border-[#d9d9d9] rounded-full justify-between items-center'>
					원하는 목적지가 없으신가요?
					<div onClick={goNewReservation} className='w-[50px] h-[50px]'>
						<img src={ReservationButton} alt='신규 예약하기' />
					</div>
				</div>
				<div className='w-[100%] flex justify-center'>
					<div className='fixed w-[100%] flex top-[15%] font-[#a9a9a9] justify-between items-center text-center'>
						<div className='w-[20%] h-[5%]'>출발지</div>
						<div className='w-[20%] h-[5%]'>도착지</div>
						<div className='w-[20%] h-[5%]'>출발 일자</div>
						<div className='w-[20%] h-[5%]'>출발 시각</div>
						<div className='w-[20%] h-[5%]'>인원 현황</div>
					</div>
					{requestArray.length !== 0 ? (
						requestArray.map((e) => {
							return <ReservationListElement reservationContent={e} />;
						})
					) : (
						<div>예약 가능한 내역이 없습니다.</div>
					)}
				</div>
			</div>
			<BottomNav />
		</>
	);
};
