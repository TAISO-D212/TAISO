import { useEffect, useState } from 'react';
import FmdGoodIcon from '@mui/icons-material/FmdGood';
import CalendarMonthIcon from '@mui/icons-material/CalendarMonth';
import AccessTimeIcon from '@mui/icons-material/AccessTime';
import { SvgLocationMarker } from '../../../assets/svg/SvgLocationMarker';

interface IReservationListElementProps {
	reservationContent: {
		rsvId: number;
		placeId: number;
		latitude: number;
		longitude: number;
		address: string;
		time: string;
		arrivalTime?: string;
		stopCnt: number;
		cnt: number;
	};
}

export const ReservationListElement = (props: IReservationListElementProps) => {
	// const navigate = useNavigate();
	// const [pageNum, setPageNum] = useState(0);

	// useEffect(() => {
	// 	console.log(props.reservationContent);
	// 	setPageNum(props.reservationContent.pid);
	// }, []);
	const onClick = () => {};

	return (
		<div
			onClick={onClick}
			className='w-[90%] flex justify-evenly items-center px-3 my-5 rounded-md shadow-[5px_15px_40px_-15px_rgba(0,0,0,0.5)]'>
			{/* <div className='flex flex-col w-[50%]'>
				<div className='flex items-center pt-2'>
					<SvgLocationFrom width='24' height='64' />
					<div className='px-5'>{props.reservationContent.startName}</div>
				</div>
			</div> */}
			<div className='flex flex-col justify-between w-[100%]'>
				<div className='flex items-center'>
					<FmdGoodIcon sx={{ color: '#85a5fc' }} />
					<div className='px-2'>{props.reservationContent.address}</div>
				</div>
				<div className='flex my-1'>
					<CalendarMonthIcon sx={{ color: '#5a85fc' }} />
					<div className='px-2'>
						{`${props.reservationContent.time.substring(0, 4)}.${props.reservationContent.time.substring(5, 7)}.${props.reservationContent.time.substring(8, 10)}`}
					</div>
				</div>
				<div className='flex my-1'>
					<CalendarMonthIcon sx={{ color: '#5a85fc' }} />
					<div className='px-2'>
						{`${props.reservationContent.time.substring(11, 13)}:${props.reservationContent.time.substring(14, 16)}`}
					</div>
				</div>
				<div className='flex my-1'>
					<AccessTimeIcon sx={{ color: '#85a5fc' }} />
					<div className='px-2'>{`${props.reservationContent.cnt}명 / 4명`}</div>
				</div>
			</div>
		</div>
	);
};
