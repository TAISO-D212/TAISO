import { getBookmarkList } from '../../apis/bookmarkApi';
import { deleteMember, getMember } from '../../apis/memberApi';
import { getRsvList } from '../../apis/reservationApi';
import { BackButton } from '../../components/BackButton';
import { BottomNav } from '../../components/BottomNav';
import useCustomLogin from '../../hooks/useCustomLogin';

export const UserPage = () => {
	const { doLogout, moveToLogin } = useCustomLogin();
	// 이거 활성화 하면 밑의 return부분 렌더링 되는 와중에 실행해서 터짐.
	// if (!isLogin) {
	// 	return moveToLogin();
	// }

	const handleClickLogout = () => {
		doLogout();
		alert('로그아웃되었습니다.');
		moveToLogin();
	};

	const handleclickdelete = () => {
		const confirmed = window.confirm('정말로 회원탈퇴하시겠습니까?');
		if (confirmed) {
			deleteMember();
			alert('회원탈퇴되었습니다.');
			moveToLogin(); // 회원탈퇴 후 로그인 페이지로 이동
		}
	}

	// // 북마크 받는 테스트
	// const handleClickGetBookmark = () => {
	// 	getBookmarkList().then((res) => {
	// 		console.log(res.data);
	// 	});
	// };

	// // 예약 목록 받는 테스트
	// const handleClickGetRsvList = () => {
	// 	getRsvList().then((res) => {
	// 		console.log(res.data);
	// 	});
	// };

	// // 멤버 정보 받는 테스트
	// const handleClickGetMemberInfo = () => {
	// 	getMember().then((data) => {
	// 		console.log(data);
	// 	});
	// };

	return (
		<>
			<div className='animate-fadeIn flex flex-col'>
				<BackButton />
				<div className="justify-center text-center mt-6 font-['Pretendard-Bold'] text-[26px]">
					내 정보
				</div>
				<div className='ml-10 mt-8 space-y-3'>
					<p>이름: 김복순</p>
					<p>email: user1@ssafy.com</p>
					<hr className="my-4 border border-t border-violet-400 mr-10" /> 
					<p onClick={handleClickLogout}>로그아웃</p>
					<p onClick={handleclickdelete}>회원탈퇴</p>
					<hr className="my-4 border border-t border-violet-400 mr-10" /> 
				</div>
			</div>
			<BottomNav />
		</>
	);
};
