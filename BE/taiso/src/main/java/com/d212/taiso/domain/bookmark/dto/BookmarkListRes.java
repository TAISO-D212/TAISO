package com.d212.taiso.domain.bookmark.dto;

import com.d212.taiso.domain.place.entity.Place;
import lombok.AllArgsConstructor;
import lombok.Data;

@AllArgsConstructor
@Data
public class BookmarkListRes {

    // 장소 이름 (유저가 정한)
    private String name;

    // 장소 객체
    private Place place;
}
