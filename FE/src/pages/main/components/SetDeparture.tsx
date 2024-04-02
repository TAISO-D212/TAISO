import { BackButton } from '../../../components/BackButton';
import { useState, useEffect } from 'react';
import { useNavigate } from 'react-router-dom';
import MapIcon from '@mui/icons-material/Map';
import { getBookmarkList } from '../../../apis/bookmarkApi';
import { BookmarkType } from '../../../interfaces/Bookmark';
import { FavoriteListElement } from '../../favorite/components/FavoriteListElement';

export const SetDeparture = () => {
	const navigate = useNavigate();
	const [bookmarkList, setBookmarkList] = useState<BookmarkType[]>([]);
	const [editMode] = useState(false);
	const [isPlaceStartSetting, setIsPlaceStartSetting] = useState(false);
	const [isPlaceEndSetting] = useState(false);
	// bookmark 정보를 가져오는 함수
	const handleGetBookmarkList = () => {
		getBookmarkList().then((res) => {
			const bookmarkData = res.data;
			console.log(bookmarkData);
			setBookmarkList(bookmarkData);
		});
	};

	useEffect(() => {
		handleGetBookmarkList();
		setIsPlaceStartSetting(true);
		return () => {
			setIsPlaceStartSetting(false);
		};
	}, []);

	const handleSetDepartureByMap = () => {
		navigate('/setDepartureByMap');
	};
	return (
		<>
			<div className='fixed z-10 top-0 w-[100%] h-[15%] flex-col justify-evenly items-end bg-white'>
				<div className='flex'>
					<BackButton />
					<div className='fixed flex justify-center w-[100%] top-[3%] mx-[5px] font-["Pretendard-Bold"] text-[25px]'>
						<div>출발 장소 찾기</div>
					</div>
				</div>
			</div>
			<div className='relative top-[15%] w-[100%] h-[70%] overflow-y-scroll'>
				{bookmarkList.map((bookmark) => (
					<FavoriteListElement
						key={bookmark.bookmarkId}
						{...bookmark}
						editMode={editMode}
						isPlaceStartSetting={isPlaceStartSetting}
						isPlaceEndSetting={isPlaceEndSetting}
						onClickDelete={null}
					/>
				))}
			</div>
			<div className='fixed z-10 bottom-0 w-[100%] h-[15%] flex flex-col justify-center items-center bg-white'>
				<div
					className='w-[70%] font-["Pretendard-Bold"] flex justify-evenly items-center text-[26px] my-3 hover:cursor-pointer'
					onClick={handleSetDepartureByMap}>
					<MapIcon sx={{ color: '#d9d9d9' }} />
					지도에서 설정하기
				</div>
				{/* <div className=' w-[70%] h-[40%] flex justify-center items-center bg-[#3422F2] rounded-full'>
					<span className='flex justify-center w-[100%] font-["Pretendard-Bold"] text-[25px] text-white'>
						출발지 설정
					</span>
				</div> */}
			</div>
		</>
	);
};
