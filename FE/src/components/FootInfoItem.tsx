interface IContent {
	title: string;
	value: string | number;
}
export default function FootInfoItem({ title, value }: IContent) {
	return (
		<div className='w-[100%] h-[100%] flex flex-col justify-center items-center'>
			<p className='text-[20px] font-["Pretendard-Bold"]'>{value}</p>
			<p className='text-[16px] text-gray'>{title}</p>
		</div>
	);
}
