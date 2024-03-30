import { useEffect, useState } from 'react';
import { BookmarkType } from '../../../interfaces/Bookmark';
import { getBookmarkList } from '../../../apis/bookmarkApi';

export const FavoriteListElement = () => {
	const [bookmarkList, setBookmarkList] = useState<BookmarkType[]>([]);
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

  // 삭제할 때는 bookmarkId를, 예약 때 위치정보 줄 시에는 placeId를 줘야합니다!!! -> 추가할 때만 해주면 되나??
  // 백에서 수정을 해야되겠는데... -> 수정해서 이제 bookmarkId만 이용한다고 보면 됨 

	return (
		<>
      <ul>
        {bookmarkList.map((bookmark) => (
          <li key={bookmark.bookmarkId}>
            <p>이름: {bookmark.name}</p>
            <p>주소: {bookmark.place.address}</p>
          </li>
        ))}
      </ul>
		</>
	);
};
