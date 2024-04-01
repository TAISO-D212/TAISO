import { deleteBookmark } from '../../../apis/bookmarkApi';
import favoriteStar from '../../../assets/icon/favorite_star.png';
import { useState } from 'react';
import { BookmarkType } from '../../../interfaces/Bookmark';
import NewReservationStore from '../../../store/NewReservationStore';
import { useNavigate } from 'react-router-dom';
import ArrowForwardIosIcon from '@mui/icons-material/ArrowForwardIos';

interface FavoriteListElementProps extends BookmarkType {
	editMode: boolean;
	isPlaceStartSetting: boolean;
	isPlaceEndSetting: boolean;
}

export const FavoriteListElement = ({
	bookmarkId,
	name,
	place,
	editMode,
	isPlaceStartSetting,
	isPlaceEndSetting,
}: FavoriteListElementProps) => {
	const handleDeleteBookmark = (bookmarkId: number) => {
		deleteBookmark(bookmarkId);
		// 새로고침해야 사라지는 문제가 있음..
		// 모달창 띄우고 거기서 확인 누를때마다 새로고침 되도록 구현하기.
	};

	const navigate = useNavigate();

	const {
		startBookmarkId,
		setStartBookmarkId,
		startLatitude,
		setStartLatitude,
		startLongitude,
		setStartLongitude,
		startAddress,
		setStartAddress,
		endBookmarkId,
		setEndBookmarkId,
		endLatitude,
		setEndLatitude,
		endLongitude,
		setEndLongitude,
		endAddress,
		setEndAddress,
	} = NewReservationStore();

	const handleStartPlaceSetting = () => {
		// 장소 설정 기능
		setStartBookmarkId(bookmarkId);
		setStartLatitude(place.latitude);
		setStartLongitude(place.longitude);
		setStartAddress(place.address);
		navigate('/reservation/new');
	};

	const handleEndPlaceSetting = () => {
		// 장소 설정 기능
		setEndBookmarkId(bookmarkId);
		setEndLatitude(place.latitude);
		setEndLongitude(place.longitude);
		setEndAddress(place.address);
		navigate('/reservation/new');
	};

	return (
		<ul>
			<li className='ml-8 border-t border-violet-200'>
				<div className='flex items-center justify-between'>
					<div
						className='flex items-center'
						onClick={
							isPlaceStartSetting
								? handleStartPlaceSetting
								: isPlaceEndSetting
									? handleEndPlaceSetting
									: null
						}>
						<div className='mr-2'>
							<img className='w-6 h-6 opacity-50' src={favoriteStar} alt='star' />
						</div>
						<div>
							<p className='mt-3 font-bold'>{name}</p>
							<p className='mb-3 text-gray-400'>{place.address}</p>
						</div>
					</div>
					{editMode && (
						<button className='mr-8 text-red-500' onClick={() => handleDeleteBookmark(bookmarkId)}>
							삭제
						</button>
					)}
					{isPlaceStartSetting && (
						<button className='mr-8 text-green-500' onClick={handleStartPlaceSetting}>
							<ArrowForwardIosIcon sx={{ color: '#C4B5FC' }} />
						</button>
					)}
					{isPlaceEndSetting && (
						<button className='mr-8 text-blue-500' onClick={handleEndPlaceSetting}>
							<ArrowForwardIosIcon sx={{ color: '#C4B5FC' }} />
						</button>
					)}
				</div>
			</li>
		</ul>
	);
};
