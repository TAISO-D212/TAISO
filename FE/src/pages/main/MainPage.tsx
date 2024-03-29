import { useEffect, useState } from 'react';
import LoadingBus from '../../assets/loading/loading.gif';
import { KakaoMap } from './components/KakaoMap';
import { BottomNav } from '../../components/BottomNav';
import { MainBottom } from './components/MainBottom';
import { CurLocCar } from '../reservation/components/CurLocCar';
// import { SetCurrentLoc } from './components/SetCurrentLoc';

export const MainPage = () => {
	const [loading, setLoading] = useState(true);

	useEffect(() => {
		setTimeout(() => {
			setLoading((prev) => !prev);
		}, 2000);
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
