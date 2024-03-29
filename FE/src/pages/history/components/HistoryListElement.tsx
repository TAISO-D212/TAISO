import MyLocationIcon from '@mui/icons-material/MyLocation';
import FmdGoodIcon from '@mui/icons-material/FmdGood';
import CalendarMonthIcon from '@mui/icons-material/CalendarMonth';
import AccessTimeIcon from '@mui/icons-material/AccessTime';
import AccessTimeFilledIcon from '@mui/icons-material/AccessTimeFilled';

interface IHistoryListElementProps {
	historyContent: {
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
	};
}

export const HistoryListElement = (props: IHistoryListElementProps) => {
	return (
		<div className='w-[90%] flex shadow-inner'>
			<div className='flex flex-col my-2 w-[100%]'>
				<div className='flex py-[3px] mx-5'>
					<CalendarMonthIcon sx={{ color: '#5a85fc' }} />
					<div className='px-2 font-["Pretendard-Bold"]'>{`${props.historyContent.departureTime.substr(0, 4)}년 ${props.historyContent.departureTime.substr(5, 2)}월 ${props.historyContent.departureTime.substr(8, 2)}일`}</div>
				</div>
				<div className='flex items-center py-[3px] mx-10'>
					<MyLocationIcon sx={{ color: '#b5c8fc' }} />
					<div className='px-2'>{props.historyContent.startAddress}</div>
				</div>
				<div className='flex items-center py-[3px] mx-10'>
					<FmdGoodIcon sx={{ color: '#85a5fc' }} />
					<div className='px-2'>{props.historyContent.endAddress}</div>
				</div>
				<div className='flex items-center py-[3px] mx-10'>
					<AccessTimeIcon sx={{ color: '#85a5fc' }} />
					<div className='px-2'>{`${props.historyContent.departureTime.substr(11, 2)}시 ${props.historyContent.departureTime.substr(14, 2)}분`}</div>
				</div>
				<div className='flex items-center py-[3px] mx-10'>
					<AccessTimeFilledIcon sx={{ color: '#b5c8fc' }} />
					<div className='px-2'>{`${props.historyContent.ArrivalTime.substr(11, 2)}시 ${props.historyContent.ArrivalTime.substr(14, 2)}분`}</div>
				</div>
			</div>
		</div>
	);
};
