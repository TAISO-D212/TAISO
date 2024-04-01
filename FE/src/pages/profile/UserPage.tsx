import { useEffect, useState } from 'react';
import { deleteMember, getMember } from '../../apis/memberApi';
import { BackButton } from '../../components/BackButton';
import { BottomNav } from '../../components/BottomNav';
import useCustomLogin from '../../hooks/useCustomLogin';
import { MemberInfo } from '../../interfaces/Member';

export const UserPage = () => {
	const { doLogout, moveToLogin } = useCustomLogin();
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
	};

	const [memberInfo, setMemberInfo] = useState<MemberInfo>(); // 회원 정보 상태값 설정

	useEffect(() => {
		// 컴포넌트가 마운트되면 회원 정보를 가져옴
		handleClickGetMemberInfo();
	}, []);

	// 멤버 정보 받는 테스트
	const handleClickGetMemberInfo = () => {
		getMember().then((res) => {
			console.log(res);
			// 응답 데이터에서 회원 정보 추출
			const memberData = res.data;

			// 추출한 회원 정보를 상태값으로 설정
			setMemberInfo(memberData);
		});
	};

	return (
		<>
			<div className='animate-fadeIn flex flex-col'>
				<BackButton />
				<div className="justify-center text-center mt-6 font-['Pretendard-Bold'] text-[26px]">
					내 정보
				</div>
				<div className='ml-8 mt-8 space-y-3'>
					<p>이름: {memberInfo?.name}</p>
					<p>이메일: {memberInfo?.email}</p>
					<hr className='my-4 border-t border-violet-200' />
					<p onClick={handleClickLogout}>로그아웃</p>
					<p onClick={handleclickdelete}>회원탈퇴</p>
					<hr className='my-4 border-t border-violet-200' />
				</div>
				{/* <button onClick={handleClickGetMemberInfo}>test</button> */}
			</div>
			<BottomNav />
		</>
	);
};
