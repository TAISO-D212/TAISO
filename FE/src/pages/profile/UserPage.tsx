import { useDispatch } from 'react-redux';
import { BottomNav } from '../../components/BottomNav';
import { logout } from '../../slices/loginSlice';

export const UserPage = () => {
	const dispatch = useDispatch();

	const handleClickLogout = () => {
		dispatch(logout());
	};

	return (
		<>
			<h1>유저의 개인정보</h1>
			<button className='rounded p-4 w-36 bg-red-500 text-xl text-black' onClick={handleClickLogout}>
				LOGOUT
			</button>
			<BottomNav />
		</>
	);
};
