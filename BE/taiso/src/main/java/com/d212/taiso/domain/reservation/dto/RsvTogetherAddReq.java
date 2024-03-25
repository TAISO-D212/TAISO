package com.d212.taiso.domain.reservation.dto;

import com.fasterxml.jackson.annotation.JsonFormat;
import lombok.Data;

import java.time.LocalDateTime;

@Data
public class RsvTogetherAddReq {

    // 이 안에 예약 관련 정보가 다 담겨 있음(도착지, 출발 시간 등)
    // 이건 path로 줄 것
//    private Long rsvId;

    // 위치 정보는 필요 (히치 하이킹 할 위치로)

    private Long placeId; // 만약에 즐겨찾기 리스트로 저장 시 (경유지 ID)

    private double latitude; // 위도

    private double longitude; // 경도

    private String address;

    private int cnt; // 예약 인원 수

}
