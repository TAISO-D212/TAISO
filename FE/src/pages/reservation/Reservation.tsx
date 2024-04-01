import { BottomNav } from '../../components/BottomNav';
import { ReservationListElement } from './components/ReservationListElement.tsx';
import { useEffect, useState } from 'react';
import { useNavigate } from 'react-router';
import { BackButton2 } from '../../components/BackButton2.tsx';
import { getRsvList } from '../../apis/reservationApi.ts';
import { RsvListType } from '../../interfaces/Reservation.ts';

export const Reservation = () => {
	const [rsvList, setRsvList] = useState<RsvListType[]>([]);
	const navigate = useNavigate();

	useEffect(() => {
		getRsvList().then((res) => {
			// setRsvList(res.data);
			console.log(res.data);
			setRsvList(res.data);
		});
	}, []);

	const goNewReservation = () => {
		navigate('/reservation/new');
	};

	return (
		<>
			<div className='flex flex-col animate-fadeIn'>
				<div className='flex ml-3 my-6 justify-between z-10 '>
					<BackButton2 />
					<div className="flex mr-14 font-['Pretendard-Bold'] text-[26px] pl-2">예약목록</div>
					<div></div>
				</div>
				<div className='flex justify-between border border-violet-200 rounded-3xl px-4 py-3 mx-6 text-lg font-medium shadow-md mb-3'>
					<p> 원하는 목적지가 없으신가요?</p>
					<button className='btn btn-sm' onClick={goNewReservation}>
						NEW
					</button>
				</div>
				{rsvList.length !== 0 ? (
					<div className='flex flex-col justify-center items-center'>
						{rsvList.map((e) => {
							return <ReservationListElement reservationContent={e} />;
						})}
					</div>
				) : (
					<div className='flex flex-col justify-center items-center font-["Pretendard-Bold"] text-[20px] mt-4'>
						예약 가능한 내역이 없습니다.
					</div>
				)}
				<BottomNav />
			</div>
		</>
	);
};
