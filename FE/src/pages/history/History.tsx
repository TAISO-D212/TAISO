import { BackButton2 } from '../../components/BackButton2';
import { BottomNav } from '../../components/BottomNav';
import { useState, useEffect } from 'react';
import { HistoryListElement } from './components/HistoryListElement';
import { getMyRsvList } from '../../apis/reservationApi';
import { MyRsvType } from '../../interfaces/Reservation';

export const History = () => {
	const [myRsv, setMyRsv] = useState<MyRsvType[]>([]);
	const [editMode, setEditMode] = useState<boolean>(false);

	useEffect(() => {
		getMyRsvList().then((res) => {
			console.log(res.data);
			setMyRsv(res.data);
		});
	}, []);

	const content = (
		<>
			<div className='w-[100%] h-[70%] flex-col justify-center animate-fadeIn'>
				{myRsv.length !== 0 ? (
					<div className=' w-[100%] h-[75%] flex flex-col justify-center items-center overflow-y-scroll'>
						{myRsv.map((e) => {
							return <HistoryListElement key={e.rsvId} rsvContent={e} editMode={editMode} />;
						})}
					</div>
				) : (
					<div className='bottom-[90px] w-[100%] h-[60%] flex flex-col justify-center items-center font-["Pretendard-Bold"] text-[20px]'>
						이용 내역이 없습니다.
					</div>
				)}
			</div>
		</>
	);

	// 편집 모드 토글 함수
	const toggleEditMode = () => {
		setEditMode(!editMode);
	};

	return (
		<>
			<div className='flex ml-3 my-6 justify-between'>
				<BackButton2 />
				<div className="flex font-['Pretendard-Bold'] text-[26px] pl-2">이용내역</div>
				<div className='mt-2 mr-8 opacity-70' onClick={toggleEditMode}>
					{editMode ? '취소' : '삭제'}
				</div>
			</div>
			<div role='tablist' className='tabs tabs-bordered w-[100%]'>
				<input
					type='radio'
					name='my_tabs_1'
					role='tab'
					className='tab'
					aria-label='예약 현황'
					checked
				/>
				<div role='tabpanel' className='w-[100%] tab-content p-5'>
					{content}
				</div>

				<input type='radio' name='my_tabs_1' role='tab' className='tab' aria-label='지난 예약' />
				<div role='tabpanel' className='w-[100%] tab-content p-5 animated-fadeIn'>
					{'지난 예약이 없습니다.'}
				</div>
			</div>
			<BottomNav />
		</>
	);
};
