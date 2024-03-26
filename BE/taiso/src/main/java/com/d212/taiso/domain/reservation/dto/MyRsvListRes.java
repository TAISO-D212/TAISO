package com.d212.taiso.domain.reservation.dto;

import java.time.LocalDateTime;
import lombok.AllArgsConstructor;
import lombok.Builder;
import lombok.Data;

/**
 * Created by 전근렬 on 2024-03-26
 */
@Data
@Builder
public class MyRsvListRes {

    private Long rsvId;

    private Long startPlaceId;

    private double startLatitude;

    private double startLongitude;

    private String startAddress;

    private Long endPlaceId;

    private double endLatitude;

    private double endLongitude;

    private String endAddress;

    private LocalDateTime time; // 예약 시작 시간

    private LocalDateTime arrivalTime; // 예약 종료 시간

    private int cnt; // 총 예약 인원
}
