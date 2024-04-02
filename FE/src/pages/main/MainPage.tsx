import { useEffect, useState } from 'react';
import LoadingBus from '../../assets/loading/loading.gif';
import { KakaoMap } from './components/KakaoMap';
import { BottomNav } from '../../components/BottomNav';
import { MainBottom } from './components/MainBottom';
// import { MovingTAISO } from '../reservation/components/MovingTAISO';
// import { SetCurrentLoc } from './components/SetCurrentLoc';
import NewReservationStore from '../../store/NewReservationStore';

export const MainPage = () => {
	const {
		setStartBookmarkId,
		setStartLatitude,
		setStartLongitude,
		setStartAddress,
		setEndBookmarkId,
		setEndLatitude,
		setEndLongitude,
		setEndAddress,
		setTime,
		setCnt,
	} = NewReservationStore();
	const [loading, setLoading] = useState(true);

	useEffect(() => {
		setStartBookmarkId(null);
		setStartLatitude(null);
		setStartLongitude(null);
		setStartAddress(null);
		setEndBookmarkId(null);
		setEndLatitude(null);
		setEndLongitude(null);
		setEndAddress(null);
		setTime(null);
		setCnt(1);
		setTimeout(() => {
			setLoading((prev) => !prev);
		}, 1000);
	}, []);

	return (
		<>
			{loading ? (
				<div className='flex justify-center items-center w-[100%] h-[100%]'>
					<img src={LoadingBus} alt='LOADING' />
				</div>
			) : (
				<>
					<KakaoMap />
					<MainBottom />
					<BottomNav />
				</>
			)}
		</>
		// <>
		// 	<CurLocCar />
		// </>
	);
};
