import { useEffect, useState } from 'react';
import { useNavigate } from 'react-router-dom';

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
		<div onClick={onClick} className='flex justify-between items-center divide-y divide-dashed hover:divide-solid'>
			<div>{props.reservationContent.startName}</div>
			<div>{props.reservationContent.endName}</div>
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
	);
};
