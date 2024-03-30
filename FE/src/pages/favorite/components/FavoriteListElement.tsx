import { deleteBookmark } from '../../../apis/bookmarkApi';
import favoriteStar from '../../../assets/icon/favorite_star.png';
import { BookmarkType } from '../../../interfaces/Bookmark';

interface FavoriteListElementProps extends BookmarkType {
	editMode: boolean;
}

export const FavoriteListElement = ({ bookmarkId, name, place, editMode }: FavoriteListElementProps) => {


	const handleDeleteBookmark = (bookmarkId: number) => {
		deleteBookmark(bookmarkId)
		// 새로고침해야 사라지는 문제가 있음..
		// 모달창 띄우고 거기서 확인 누를때마다 새로고침 되도록 구현하기.
	}

	return (
			<ul>
					<li className='ml-8 border-t border-violet-200'>
							<div className='flex items-center justify-between'>
									<div className='flex items-center'>
											<div className='mr-2'>
													<img className='w-6 h-6 opacity-50' src={favoriteStar} alt='star' />
											</div>
											<div>
													<p className='mt-3 font-bold'>{name}</p>
													<p className='mb-3 text-gray-400'>{place.address}</p>
											</div>
									</div>
									{editMode && (
											<button className='mr-8 text-red-500' onClick={() => handleDeleteBookmark(bookmarkId)}>삭제</button>
									)}
							</div>
					</li>
			</ul>
	);
};