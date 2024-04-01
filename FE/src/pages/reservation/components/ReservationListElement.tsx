import FmdGoodIcon from '@mui/icons-material/FmdGood';
import CalendarMonthIcon from '@mui/icons-material/CalendarMonth';
import AccessTimeIcon from '@mui/icons-material/AccessTime';
import PersonIcon from '@mui/icons-material/Person';

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
	const onClick = () => {};
	return (
		<>
			<div
				className='w-[90%] px-3 py-2 my-3 border border-violet-200 rounded-md shadow-lg'>
				<div className='flex flex-col'>
					<div className='flex'>
						<FmdGoodIcon sx={{ color: '#C4B5FC' }} />
						<div className='px-2'>{props.reservationContent.address}</div>
					</div>
					<div className='flex my-1'>
						<CalendarMonthIcon sx={{ color: '#C4B5FC' }} />
						<div className='px-2'>
							{`${props.reservationContent.time.substring(0, 4)}.${props.reservationContent.time.substring(5, 7)}.${props.reservationContent.time.substring(8, 10)}`}
						</div>
					</div>
					<div className='flex my-1'>
						<AccessTimeIcon sx={{ color: '#C4B5FC' }} />
						<div className='px-2'>
							{`${props.reservationContent.time.substring(11, 13)}:${props.reservationContent.time.substring(14, 16)}`}
						</div>
					</div>
					<div className='flex my-1 justify-between'>
						<div className='flex'>
							<PersonIcon sx={{ color: '#C4B5FC' }} />
							<div className='px-2'>{`${props.reservationContent.cnt}명 / 4명`}</div>
						</div>
						<button className='btn btn-sm'
						onClick={onClick}>
						합승예약
					</button>
					</div>
				</div>
			</div>
		</>
	);
};
