import { viteConfig } from './viteConfig';
import jwtAxios from '../utils/jwtUtil';
import { BookmarkInputType } from '../interfaces/Bookmark';

const host = `${viteConfig.VITE_BASE_URL}/api/bookmarks`;

// 북마크 가져오기
export const getBookmarkList = async () => {
	const res = await jwtAxios.get(`${host}/`);
	return res.data;
};

// 북마크 추가
export const addBookmark = async (bookmarkObj: BookmarkInputType) => {
	const res = await jwtAxios.post(`${host}/`, bookmarkObj);
	return res.data;
};

// 북마크 삭제
export const deleteBookmark = async (bookmarkId: number) => {
	const res = await jwtAxios.delete(`${host}/${bookmarkId}`);
	return res.data;
};
