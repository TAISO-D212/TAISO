import { deleteBookmark } from '../../../apis/bookmarkApi';
import favoriteStar from '../../../assets/icon/favorite_star.png';
import { BookmarkType } from '../../../interfaces/Bookmark';
import NewReservationStore from '../../../store/NewReservationStore';
import { useNavigate, useParams } from 'react-router-dom';
import ArrowForwardIosIcon from '@mui/icons-material/ArrowForwardIos';
import TogetherRsvStore from '../../../store/TogetherRsvStore';

interface FavoriteListElementProps extends BookmarkType {
	editMode?: boolean;
	isPlaceStartSetting?: boolean;
	isPlaceEndSetting?: boolean;
	isTogetherSetting?: boolean;
	onClickDelete?: (bookmarkId: number) => void;
}

export const FavoriteListElement = ({
	bookmarkId,
	name,
	place,
	editMode,
	isPlaceStartSetting,
	isPlaceEndSetting,
	isTogetherSetting,
	onClickDelete,
}: FavoriteListElementProps) => {
	const handleDeleteBookmark = (bookmarkId: number) => {
		deleteBookmark(bookmarkId);
		onClickDelete(bookmarkId);
	};

	const navigate = useNavigate();

	const { rsvId } = useParams(); // 문자열로 받아짐.
	const rsvIdInt = parseInt(rsvId, 10);

	const {
		setStartBookmarkId,
		setStartLatitude,
		setStartLongitude,
		setStartAddress,
		setEndBookmarkId,
		setEndLatitude,
		setEndLongitude,
		setEndAddress,
	} = NewReservationStore();

	const { setBookmarkId, setLatitude, setLongitude, setAddress } = TogetherRsvStore();

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

	// 합승 출발 장소 설정 기능
	const handleTogeterStartPlaceSetting = (rsvId: number) => {
		// 장소 설정 기능
		setBookmarkId(bookmarkId);
		setLatitude(place.latitude);
		setLongitude(place.longitude);
		setAddress(place.address);
		navigate(`/reservation/${rsvId}`);
	};

	return (
		<ul>
			<li className='ml-8 border-t border-violet-200'>
				<div className='flex items-center justify-between'>
					<div
						className='flex items-center'
						onClick={
							(isPlaceStartSetting && handleStartPlaceSetting) ||
							(isPlaceEndSetting && handleEndPlaceSetting) ||
							undefined
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
					{isTogetherSetting && (
						<button
							className='mr-8 text-pink-500'
							onClick={() => handleTogeterStartPlaceSetting(rsvIdInt)}>
							<ArrowForwardIosIcon sx={{ color: '#C4B5FC' }} />
						</button>
					)}
				</div>
			</li>
		</ul>
	);
};
