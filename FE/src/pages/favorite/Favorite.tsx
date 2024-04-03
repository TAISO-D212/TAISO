import { BackButton2 } from '../../components/BackButton2';
import { BottomNav } from '../../components/BottomNav';
import { FavoriteListElement } from './components/FavoriteListElement';
import { useEffect, useState } from 'react';
import { BookmarkType } from '../../interfaces/Bookmark';
import { getBookmarkList } from '../../apis/bookmarkApi';

export const Favorite = () => {
	const [bookmarkList, setBookmarkList] = useState<BookmarkType[]>([]);

	const [editMode, setEditMode] = useState(false);
	const [isPlaceStartSetting] = useState(false);
	const [isPlaceEndSetting] = useState(false);

	useEffect(() => {
		handleGetBookmarkList();
	}, []);

	// bookmark 정보를 가져오는 함수
	const handleGetBookmarkList = () => {
		getBookmarkList().then((res) => {
			const bookmarkData = res.data;
			console.log(bookmarkData);
			setBookmarkList(bookmarkData);
		});
	};

	// 편집 모드 토글 함수
	const toggleEditMode = () => {
		setEditMode(!editMode);
	};

	//삭제 함수
	const onClickDelete = (bookmarkId: number) => {
		setBookmarkList(bookmarkList.filter((bookmark) => bookmark.bookmarkId !== bookmarkId));
	};

	return (
		<>
			<div className='animate-fadeIn flex flex-col h-[100%]'>
				<div className='flex ml-3 my-6 justify-between'>
					<BackButton2 />
					<div className="flex font-['Pretendard-Bold'] text-[26px] pl-2">즐겨찾기</div>
					<div className='mt-2 mr-8 opacity-70' onClick={toggleEditMode}>
						{editMode ? '취소' : '편집'}
					</div>
				</div>
				<div className='h-[80%] overflow-hidden overflow-y-scroll'>
					{bookmarkList.map((bookmark) => (
						<FavoriteListElement
							key={bookmark.bookmarkId}
							{...bookmark}
							editMode={editMode}
							isPlaceStartSetting={isPlaceStartSetting}
							isPlaceEndSetting={isPlaceEndSetting}
							onClickDelete={onClickDelete}
						/>
					))}
				</div>
			</div>
			<BottomNav />
		</>
	);
};
