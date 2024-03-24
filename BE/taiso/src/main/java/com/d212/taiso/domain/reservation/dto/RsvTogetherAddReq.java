package com.d212.taiso.domain.reservation.dto;

import lombok.Data;

import java.time.LocalDateTime;

@Data
public class RsvTogetherAddReq {

    private Long rsvId;

    private LocalDateTime time; // 출발 시간

    private int cnt; // 예약 인원 수


}
