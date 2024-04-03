interface IButtonContent {
	handleMoveTrack: any;
	isMoving: boolean;
	stopMove: any;
}
export default function StopTrackingMove({ handleMoveTrack, isMoving, stopMove }: IButtonContent) {
	return (
		<div className='flex justify-evenly items-center w-[100%] h-[100%] bg-[#3422F2] rounded-full'>
			{/* <button onClick={handleMoveTrack} className='w-[40%]'>
				{isMoving ? '추적 중지' : '추적 시작'}
			</button> */}
			<button onClick={stopMove} className='w-[60%] font-["Pretendard-Bold"] text-[25px] text-white'>
				종료하기
			</button>
		</div>
	);
}
