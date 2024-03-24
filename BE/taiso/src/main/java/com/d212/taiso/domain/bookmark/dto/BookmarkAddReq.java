package com.d212.taiso.domain.bookmark.dto;

import com.d212.taiso.domain.place.entity.Place;
import jakarta.persistence.Column;
import lombok.Data;

@Data
public class BookmarkAddReq {

    // 장소 이름 (유저가 정한)
    private String name;

    private double latitude; // 위도

    private double longitude; // 경도

    private String address;
}
