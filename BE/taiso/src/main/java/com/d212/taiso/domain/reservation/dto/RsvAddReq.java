package com.d212.taiso.domain.reservation.dto;

import com.fasterxml.jackson.annotation.JsonFormat;
import lombok.Data;

import java.time.LocalDateTime;

/**
 * Created by 전근렬 on 2024-03-22
 */

@Data
public class RsvAddReq {

    private double startLatitude;
    private double startLongitude;
    private String startAddress;

    // 즐겨찾기 리스트로 추가 시
    // 근데 이거 널 값이면 안되잖아? -> db에 넣을 시에만 신경써주기
    private Long startBookmarkId;

    private double endLatitude;
    private double endLongitude;
    private String endAddress;
    private Long endBookmarkId;


    // 이렇게 해야 값이 들어가네....
    @JsonFormat(shape = JsonFormat.Shape.STRING, pattern = "yyyy-MM-dd HH:mm:ss", timezone = "Asia/Seoul")
    private LocalDateTime time; // 출발 시간

    private int cnt; // 예약 인원 수

}
