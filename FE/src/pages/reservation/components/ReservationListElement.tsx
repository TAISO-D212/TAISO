import { useEffect, useState } from 'react';
import { useNavigate } from 'react-router-dom';
import { SvgLocationFrom } from '../../../assets/svg/SvgLocationFrom';
import { SvgLocationMarker } from '../../../assets/svg/SvgLocationMarker';

interface IReservationListElementProps {
	reservationContent: {
		pid: number;
		startName: string;
		endName: string;
		startDate: string;
		currentMember: number;
		totalMember: number;
	};
}

export const ReservationListElement = (props: IReservationListElementProps) => {
	const navigate = useNavigate();
	const [pageNum, setPageNum] = useState(0);

	useEffect(() => {
		console.log(props.reservationContent);
		setPageNum(props.reservationContent.pid);
	}, []);
	const onClick = () => {
		const url = `/reservation/${pageNum}`;
		navigate(url);
	};

	return (
		<div
			onClick={onClick}
			className='w-[80%] flex justify-evenly items-center px-5 my-5 rounded-md shadow-[5px_15px_40px_-15px_rgba(0,0,0,0.5)]'>
			<div className='flex flex-col w-[50%]'>
				<div className='flex items-center pt-2'>
					<SvgLocationFrom width='24' height='64' />
					<div className='px-5'>{props.reservationContent.startName}</div>
				</div>
				<div className='flex items-center'>
					<SvgLocationMarker width='24' height='64' />
					<div className='px-5'>{props.reservationContent.endName}</div>
				</div>
			</div>
			<div className='flex flex-col justify-between items-end w-[30%]'>
				<div>{`${props.reservationContent.startDate.substr(
					0,
					4,
				)}.${props.reservationContent.startDate.substr(5, 2)}.${props.reservationContent.startDate.substr(8, 2)}`}</div>
				<div>{`${props.reservationContent.startDate.substr(
					11,
					2,
				)}:${props.reservationContent.startDate.substr(14, 2)}`}</div>
				<div>{`${props.reservationContent.currentMember}명 / ${props.reservationContent.totalMember}명`}</div>
			</div>
		</div>
	);
};
