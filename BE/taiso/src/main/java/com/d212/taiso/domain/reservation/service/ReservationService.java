package com.d212.taiso.domain.reservation.service;


import com.d212.taiso.domain.reservation.dto.RsvAddReq;
import com.d212.taiso.domain.reservation.dto.RsvListRes;
import com.d212.taiso.domain.reservation.dto.RsvTogetherAddReq;

import java.util.List;

/**
 * Created by 전근렬 on 2024-03-22
 */
public interface ReservationService {


    // 전제 예약 리스트 조회하기 (현재 시간 이후로)
    List<RsvListRes> getAllRsvList();

    // 내 예약 리스트 조회하기
    List<RsvListRes> getMyRsvList();

    // 예약 추가하기
    void addRsv(RsvAddReq rsvAddReq);

    // 예약 추가하기 (합승)
    void addTogetherRsv(Long rsvId, RsvTogetherAddReq rsvTogetherAddReq);

    // 예약 삭제하기
    // 만약에 예약을 생성한 사람이 해당 예약을 취소한다면??
    // 그럼 합승한 사람 여부를 판단하고 변경 되어야 하는데 이걸 우예 짜지


}
