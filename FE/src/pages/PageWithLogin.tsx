import useCustomLogin from '../hooks/useCustomLogin';
import { useEffect } from 'react';
import { useNavigate } from 'react-router-dom';

interface AuthProps {
	children: React.ReactNode;
}

const PageWithLogin: React.FC<AuthProps> = ({ children }) => {
	const { isLogin } = useCustomLogin();
	const navigate = useNavigate();
	useEffect(() => {
		if (!isLogin) {
			alert('로그인이 필요한 서비스입니다.');
			navigate('/');
		}
	}, []);

	if (isLogin) {
		return <>{children}</>;
	}
};

export default PageWithLogin;
