import { getBookmarkList } from '../../apis/bookmarkApi';
import { getRsvList } from '../../apis/reservationApi';
import { BottomNav } from '../../components/BottomNav';
import useCustomLogin from '../../hooks/useCustomLogin';

export const UserPage = () => {
	
	const { isLogin, doLogout, moveToLogin } = useCustomLogin();

	if (!isLogin) {
		return moveToLogin();
	}
	
	const handleClickLogout = () => {
		doLogout();
		alert("로그아웃되었습니다.")
	};

	// 북마크 받는 테스트
	const handleClickGetBookmark = () => {
		getBookmarkList().then((data) => {
			console.log(data)
		})
	}

	// 예약 목록 받는 테스트
	const handleClickGetRsvList = () => {
		getRsvList().then((data) => {
			console.log(data)
		})
	}

	return (
		<>
			<h1>유저의 개인정보</h1>
			<button className='rounded p-4 w-36 bg-red-500 text-xl text-black' onClick={handleClickLogout}>
				LOGOUT
			</button>

			<button className='rounded p-4 w-36 bg-red-500 text-xl text-black' onClick={handleClickGetBookmark}>
				북마크 Test
			</button>

			<button className='rounded p-4 w-36 bg-red-500 text-xl text-black' onClick={handleClickGetRsvList}>
				예약목록 Test
			</button>
			<BottomNav />
		</>
	);
};
