package com.d212.taiso.domain.reservation.dto;

import java.time.LocalDateTime;

/**
 * Created by 전근렬 on 2024-03-22
 */
public class RsvAddReq {

    private double latitude;
    private double longitude;

    // 즐겨찾기 리스트로 추가 시
    // 근데 이거 널 값이면 안되잖아? -> db에 넣을 시에만 신경써주기
    private Long placeId;
    private LocalDateTime time; // 출발 시간

    private int cnt; // 예약 인원 수
    // 예약 인원 수

    // 예약 아이디인데 이건 나중에 서비스 쪽에서 따로 처리하기
    private Long rsvId;
}
