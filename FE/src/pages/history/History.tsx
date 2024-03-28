import { BackButton } from '../../components/BackButton';
import { BottomNav } from '../../components/BottomNav';
import { useState, useEffect } from 'react';
import { HistoryListElement } from './components/HistoryListElement';

interface IRsvHistoryType {
	startPlaceId?: number;
	startLatitude?: number;
	startLongitude?: number;
	startAddress?: string;
	endPlaceId?: number;
	endLatitude?: number;
	endLongitude?: number;
	endAddress?: string;
	departureTime: string;
	ArrivalTime: string;
	cnt: number;
}

export const History = () => {
	const [historyArr, setHistoryArr] = useState<IRsvHistoryType[]>([]);
	useEffect(() => {
		// TODO : 더미데이터 만들어서 내역 보여주기!
		setHistoryArr([
			{
				startPlaceId: 1,
				startLatitude: 37.5665,
				startLongitude: 126.978,
				startAddress: '경상북도 구미시 진평동',
				endPlaceId: 2,
				endLatitude: 37.5665,
				endLongitude: 126.978,
				endAddress: '경상북도 구미시 인의동',
				departureTime: '2022-01-01T00:00:00.000Z',
				ArrivalTime: '2022-01-01T00:12:30.000Z',
				cnt: 1,
			},
			{
				startPlaceId: 3,
				startLatitude: 37.5665,
				startLongitude: 126.978,
				startAddress: '대구광역시 수성구 두산동',
				endPlaceId: 4,
				endLatitude: 37.5665,
				endLongitude: 126.978,
				endAddress: '대구광역시 수성구 만촌동',
				departureTime: '2022-01-01T00:00:00.000Z',
				ArrivalTime: '2022-01-01T00:03:44.000Z',
				cnt: 1,
			},
			{
				startPlaceId: 5,
				startLatitude: 37.5665,
				startLongitude: 126.978,
				startAddress: '서울특별시 관악구 신림동',
				endPlaceId: 6,
				endLatitude: 37.5665,
				endLongitude: 126.978,
				endAddress: '서울특별시 관악구 대학동',
				departureTime: '2022-01-01T00:00:00.000Z',
				ArrivalTime: '2022-01-01T00:23:10.000Z',
				cnt: 1,
			},
			{
				startPlaceId: 5,
				startLatitude: 37.5665,
				startLongitude: 126.978,
				startAddress: '서울특별시 강남구 역삼동',
				endPlaceId: 6,
				endLatitude: 37.5665,
				endLongitude: 126.978,
				endAddress: '서울특별시 강남구 신사동',
				departureTime: '2022-01-01T00:00:00.000Z',
				ArrivalTime: '2022-01-01T00:47:10.000Z',
				cnt: 1,
			},
		]);
	}, []);

	return (
		<>
			<BackButton />
			<div className='fixed w-[100%] h-[12%] top-0'>
				<div className='w-[100%] h-[100%] flex flex-col font-[#a9a9a9] justify-center items-center text-center'>
					<div className="font-['Pretendard-Bold'] text-[26px] my-[2%]">이용내역</div>
				</div>
			</div>
			<div className='fixed w-[100%] flex-col justify-center'>
				{historyArr.length !== 0 ? (
					<div className='fixed bottom-[90px] w-[100%] h-[75%] flex flex-col justify-center items-center divide-y divide-dashed divide-slate-300 overflow-hidden'>
						{historyArr.map((e) => {
							return <HistoryListElement historyContent={e} />;
						})}
					</div>
				) : (
					<div className='bottom-[90px] w-[100%] h-[60%] flex flex-col justify-center items-center font-["Pretendard-Bold"] text-[20px]'>
						이용 내역이 없습니다.
					</div>
				)}
			</div>
			<BottomNav />
		</>
	);
};
