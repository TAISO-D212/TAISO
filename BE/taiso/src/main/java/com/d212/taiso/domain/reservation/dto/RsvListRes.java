package com.d212.taiso.domain.reservation.dto;

import java.time.LocalDateTime;

import com.d212.taiso.domain.place.entity.Place;
import lombok.Data;


/**
 * Created by 전근렬 on 2024-03-22
 */

// Todo
@Data
public class RsvListRes {

    private Long rsvId;

    // 위치(목적지) 정보
    private Place place;

    private LocalDateTime time; // 예약 일시 (출발 시간)

    private LocalDateTime arrivalTime; // 예상 종료 시간

    private int stopCnt; // 경유지 수

    private int cnt; // 총 예약 인원 (예약 별)

//    private String routeImg; // 총 경로 이미지


}
