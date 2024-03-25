package com.d212.taiso.domain.bookmark.dto;

import com.d212.taiso.domain.place.entity.Place;
import lombok.AllArgsConstructor;
import lombok.Data;

@AllArgsConstructor
@Data
public class BookmarkListRes {

    // 북마크 id
    private Long bookmarkId;

    // 장소 이름 (유저가 정한)
    private String name;

    // 장소 객체
    // Todo 여긴 또 왜 됨 (객체인데...)??
    private Place place;
}
