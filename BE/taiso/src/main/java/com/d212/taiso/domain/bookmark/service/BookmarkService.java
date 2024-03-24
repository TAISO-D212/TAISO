package com.d212.taiso.domain.bookmark.service;

import com.d212.taiso.domain.bookmark.dto.BookmarkAddReq;
import com.d212.taiso.domain.bookmark.dto.BookmarkListRes;

import java.util.List;

public interface BookmarkService {

    // 해당 유저가 등록한 북마크 리스트 가져오기
    List<BookmarkListRes> getMyBookmarkList();

    // 북마크 등록
    void addBookmark(BookmarkAddReq bookmarkAddReq);

    // 북마크 삭제
    void deleteBookmark(Long bookmarkId);
}
