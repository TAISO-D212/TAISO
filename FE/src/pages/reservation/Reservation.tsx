import { BottomNav } from '../../components/BottomNav';
import { ReservationListElement } from './components/ReservationListElement.tsx';
import { useEffect, useState } from 'react';
import { useNavigate } from 'react-router';
import { BackButton2 } from '../../components/BackButton2.tsx';

interface IReservationItem {
	pid: number;
	startName: string;
	endName: string;
	startDate: string;
	currentMember: number;
	totalMember: number;
}

export const Reservation = () => {
	const [requestArray, setRequestArray] = useState<IReservationItem[]>([]);
	const navigate = useNavigate();

	useEffect(() => {
		// dummy data
		setRequestArray([
			{
				pid: 1,
				startName: '출발지',
				endName: '도착지',
				startDate: '2022-01-01T00:00:00.000Z',
				currentMember: 1,
				totalMember: 4,
			},
			{
				pid: 2,
				startName: '출발지',
				endName: '도착지',
				startDate: '2022-01-01T00:00:00.000Z',
				currentMember: 1,
				totalMember: 4,
			},
			{
				pid: 3,
				startName: '출발지',
				endName: '도착지',
				startDate: '2022-01-01T00:00:00.000Z',
				currentMember: 1,
				totalMember: 4,
			},
			{
				pid: 4,
				startName: '출발지',
				endName: '도착지',
				startDate: '2022-01-01T00:00:00.000Z',
				currentMember: 1,
				totalMember: 4,
			},
		]);
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
				<div className='flex justify-between border border-violet-200 rounded-3xl px-4 py-3 mx-6 text-lg font-medium shadow-md '>
					<p> 원하는 목적지가 없으신가요?</p>
					<button className='btn btn-sm' onClick={goNewReservation}>
						NEW
					</button>
				</div>
				{requestArray.length !== 0 ? (
					<div>
						{requestArray.map((e) => {
							return <ReservationListElement reservationContent={e} />;
						})}
					</div>
				) : (
					<div className='bottom-[90px] w-[100%] h-[60%] flex flex-col justify-center items-center font-["Pretendard-Bold"] text-[20px]'>
						예약 가능한 내역이 없습니다.
					</div>
				)}
				<BottomNav />
			</div>
		</>
	);
};
