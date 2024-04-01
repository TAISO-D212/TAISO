import { BottomNav } from '../../components/BottomNav';
import { ReservationListElement } from './components/ReservationListElement.tsx';
import { useEffect, useState } from 'react';
import { useNavigate } from 'react-router';
import { BackButton } from '../../components/BackButton.tsx';
import ReservationButton from '../../assets/image/reservation_button.png';
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
			<div className='fixed w-[100%] h-[20%] flex font-[#a9a9a9] justify-center text-center bg-white z-10 animate-fadeIn'>
				<BackButton />
				<div className="fixed top-[2%] font-['Pretendard-Bold'] text-[26px] my-[2%]">예약목록</div>
				<div className='fixed bottom-[90px] w-[100%] flex flex-col justify-center items-center'>
					<div className='fixed flex top-[10%] w-[90%] h-[8%] px-5 shadow-md border border-[#d9d9d9] rounded-full justify-between items-center'>
						원하는 목적지가 없으신가요?
						<div onClick={goNewReservation} className='w-[45px] h-[45px]'>
							<img src={ReservationButton} alt='신규 예약하기' />
						</div>
					</div>
				</div>
			</div>

			{rsvList.length !== 0 ? (
				<div className='fixed bottom-[90px] w-[100%] h-[74%] pt-60 flex flex-col justify-center items-center animate-fadeIn overflow-hidden overflow-y-scroll'>
					{rsvList.map((e) => {
						return <ReservationListElement reservationContent={e} />;
					})}
				</div>
			) : (
				<div className='bottom-[90px] w-[100%] h-[60%] flex flex-col justify-center items-center font-["Pretendard-Bold"] text-[20px]'>
					예약 가능한 내역이 없습니다.
				</div>
			)}

			<BottomNav />
		</>
	);
};
