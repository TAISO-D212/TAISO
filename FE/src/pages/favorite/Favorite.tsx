import { BackButton } from "../../components/BackButton"
import { BottomNav } from "../../components/BottomNav"

export const Favorite = () => {
  return (
    <>
			<div className='animate-fadeIn flex flex-col'>
				<BackButton />
				<div className="justify-center text-center mt-6 font-['Pretendard-Bold'] text-[26px]">
					즐겨찾기
				</div>
			</div>
			<BottomNav />
		</>
  )
}