package com.d212.taiso.domain.bookmark.controller;


import com.d212.taiso.domain.bookmark.dto.BookmarkAddReq;
import com.d212.taiso.domain.bookmark.dto.BookmarkListRes;
import com.d212.taiso.domain.bookmark.service.BookmarkService;
import com.d212.taiso.global.result.ResultCode;
import com.d212.taiso.global.result.ResultResponse;
import lombok.RequiredArgsConstructor;
import lombok.extern.log4j.Log4j2;
import org.springframework.http.ResponseEntity;
import org.springframework.web.bind.annotation.*;

import java.util.List;

@Log4j2
@RestController
@RequestMapping("/bookmarks")
@RequiredArgsConstructor
public class BookmarkController {

    private final BookmarkService bookmarkService;

    @GetMapping("/")
    public ResponseEntity<ResultResponse> getMyBookmarks() {
        List<BookmarkListRes> myBookmarkList = bookmarkService.getMyBookmarkList();
        return ResponseEntity.ok(ResultResponse.of(ResultCode.GET_BOOKMARK_SUCCESS, myBookmarkList));
    }

    @PostMapping("/")
    public ResponseEntity<ResultResponse> addBookmark(@RequestBody BookmarkAddReq bookmarkAddReq) {
        bookmarkService.addBookmark(bookmarkAddReq);
        return ResponseEntity.ok(ResultResponse.of(ResultCode.ADD_BOOKMARK_SUCCESS, true));
    }

    @DeleteMapping("/{bookmarkId}")
    public ResponseEntity<ResultResponse> deleteBookmark(@PathVariable("bookmarkId") Long bookmarkId) {
        bookmarkService.deleteBookmark(bookmarkId);
        return ResponseEntity.ok(ResultResponse.of(ResultCode.DELETE_BOOKMARK_SUCCESS, true));
    }

}
