interface IButtonContent {
	handleMoveTrack: any;
	isMoving: boolean;
	stopMove: any;
}
export default function StopTrackingMove({ handleMoveTrack, isMoving, stopMove }: IButtonContent) {
	return (
		<div className='flex justify-evenly items-center w-[100%] h-[100%]'>
			<button onClick={handleMoveTrack} className='w-[40%]'>
				{isMoving ? '추적 중지' : '추적 시작'}
			</button>
			<button onClick={stopMove} className='w-[40%]'>
				RESET
			</button>
		</div>
	);
}
