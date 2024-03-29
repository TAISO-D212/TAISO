import { viteConfig } from './viteConfig';
import jwtAxios from '../utils/jwtUtil';
import { BookmarkInputType, BookmarkListType } from '../interfaces/Bookmark';
import { APIResponse } from '../interfaces/Index';

const host = `${viteConfig.VITE_BASE_URL}/api/bookmarks`;

// 북마크 가져오기
export const getBookmarkList = async ():Promise<APIResponse<BookmarkListType>> => {
	const res = await jwtAxios.get(`${host}/`);
	return res.data;
};

// 북마크 추가
export const addBookmark = async (bookmarkObj: BookmarkInputType): Promise<APIResponse<boolean>> => {
	const res = await jwtAxios.post(`${host}/`, bookmarkObj);
	return res.data;
};

// 북마크 삭제
export const deleteBookmark = async (bookmarkId: number):Promise<APIResponse<boolean>> => {
	const res = await jwtAxios.delete(`${host}/${bookmarkId}`);
	return res.data;
};
