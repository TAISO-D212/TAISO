import { KakaoMap } from './components/KakaoMap';
import { BottomNav } from '../../components/BottomNav';
import { MainBottom } from './components/MainBottom';
// import { SetCurrentLoc } from './components/SetCurrentLoc';

export const MainPage = () => {
	return (
		<>
			<KakaoMap />
			<MainBottom />
			<BottomNav />
		</>
	);
};
