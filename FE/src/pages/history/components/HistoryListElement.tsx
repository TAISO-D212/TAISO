import MyLocationIcon from '@mui/icons-material/MyLocation';
import FmdGoodIcon from '@mui/icons-material/FmdGood';
import CalendarMonthIcon from '@mui/icons-material/CalendarMonth';
import AccessTimeIcon from '@mui/icons-material/AccessTime';
// import AccessTimeFilledIcon from '@mui/icons-material/AccessTimeFilled';
import { deleteRsv } from '../../../apis/reservationApi';

interface IRsvListElementProps {
	rsvContent: {
		rsvId: number;
		startPlaceId: number;
		startLatitude: number;
		startLongitude: number;
		startAddress: string;
		endPlaceId: number;
		endLatitude: number;
		endLongitude: number;
		endAddress: string;
		time: string;
		arrivalTime?: string;
		cnt: number;
	};
	editMode: boolean;
	onClickDelete: (rsvId: number) => void;
}

export const HistoryListElement = ({
	rsvContent,
	editMode,
	onClickDelete,
}: IRsvListElementProps) => {
	const handleCancelRsv = (rsvId: number, placeId: number) => {
		console.log('예약 ID', rsvId, '출발지 ID', placeId);
		deleteRsv(rsvId, placeId).then((res) => {
			console.log(res);
		});
		onClickDelete(rsvContent.rsvId);
	};

	const endTime =
		Number(rsvContent?.time.substring(11, 13)) / 10 < 1
			? '0' + String(Number(rsvContent?.time.substring(11, 13)) + 1)
			: Number(rsvContent?.time.substring(11, 13)) + 1;

	return (
		<div className='w-[90%] px-3 py-2 my-3 border border-violet-200 rounded-md shadow-lg'>
			<div className='flex flex-col w-[100%]'>
				<div className='flex'>
					<CalendarMonthIcon sx={{ color: '#C4B5FC' }} />
					<div className='px-2 font-["Pretendard-Bold"]'>{`${rsvContent.time.substring(0, 4)}년 ${rsvContent.time.substring(5, 7)}월 ${rsvContent.time.substring(8, 10)}일`}</div>
				</div>
				<div className='flex my-1'>
					<MyLocationIcon sx={{ color: '#C4B5FC' }} />
					<div className='px-2'>{rsvContent.startAddress}</div>
				</div>
				<div className='flex my-1'>
					<FmdGoodIcon sx={{ color: '#C4B5FC' }} />
					<div className='px-2'>{rsvContent.endAddress}</div>
				</div>
				<div className='flex my-1 justify-between'>
					<div className='flex'>
						<AccessTimeIcon sx={{ color: '#C4B5FC' }} />
						<div className='px-2'>{`${rsvContent?.time?.substring(11, 13)}시 ~ ${endTime}시`}</div>
					</div>
					{editMode && (
						<button
							className='mr-8 text-red-500'
							onClick={() => handleCancelRsv(rsvContent.rsvId, rsvContent.startPlaceId)}>
							삭제
						</button>
					)}
				</div>
				{/* <div className='flex my-1'>
					<AccessTimeFilledIcon sx={{ color: '#C4B5FC' }} />
					<div className='px-2'>{`${rsvContent?.arrivalTime?.substring(12, 14)}시 ${rsvContent?.arrivalTime?.substring(15, 17)}분`}</div>
				</div> */}
			</div>
		</div>
	);
};
