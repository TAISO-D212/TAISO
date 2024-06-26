import { FaBusSimple } from 'react-icons/fa6';
import { useNavigate } from 'react-router-dom';
import useCustomLogin from '../../../hooks/useCustomLogin';
export const MainBottom = () => {
	const navigate = useNavigate();
	const { loginState } = useCustomLogin();
	const userName = loginState.name;

	const goSetDeparture = () => {
		navigate('/setDeparture');
	};

	const goSetArrival = () => {
		navigate('/setArrival');
	};
	return (
		<>
			<div className='fixed bottom-0 w-[100%] h-[35%] z-10 backdrop-blur-sm bg-white rounded-2xl flex flex-col items-center border-slate-200 rounded-t-[12px]'>
				<div className='w-[80%] font-["Pretendard-Bold"] text-[20px] mt-5'>
					{userName} 님 안녕하세요
				</div>
				{/* <div className='w-[80%] h-[36%] backdrop-blur-sm border-2 border-[#C4B5FC] rounded-lg divide-y divide-dashed divide-[#C4B5FC] my-7'> */}
				<div className='w-[80%] h-[36%] flex justify-center items-center font-["Pretendard-Bold"] bg-[#ebe8f9] text-[20px] border-2 border-[#C4B5FC] rounded-lg my-7'>
					{/* <div className='h-[50%] flex items-center' onClick={goSetDeparture}>
						<span className='mx-[5px]'>
							<FaBusSimple color='navy' className='mx-[5px]' />
						</span>
						<span className='mx-[5px]'>출발지 : {}</span>
					</div>
					<div className='h-[50%] flex items-center' onClick={goSetArrival}>
						<span className='mx-[5px]'>
							<FaBusSimple color='skyblue' className='mx-[5px]' />
						</span>
						<span className='mx-[5px]'>목적지 : {}</span>
						<span></span>
					</div> */}
					<div className='flex justify-center w-[100%]'>
						<button
							className='w-[70%] btn bg-white font-["Pretendard-Bold"] text-[25px] text-[#917FFB] shadow-lg'
							onClick={() => {
								navigate('/move');
							}}>
							차량 탑승 확인
						</button>
					</div>
				</div>
			</div>
		</>
	);
};
