package com.d212.taiso.domain.reservation.service;


import com.d212.taiso.domain.reservation.dto.RsvAddReq;
import com.d212.taiso.domain.reservation.dto.RsvListRes;
import java.util.List;

/**
 * Created by 전근렬 on 2024-03-22
 */
public interface ReservationService {

    // 전제 예약 리스트 조회하기
    List<RsvListRes> getAllRsvList();

    // 내 예약 리스트 조회하기
    List<RsvListRes> getMyRsvList();

    // 예약 추가하기
    void addRsv(RsvAddReq rsvAddReq);
    // 예약 추가하기 (합승)

    void addRsvCarpool(RsvAddReq rsvAddReq);

    // 예약 삭제하기 -> 이거 두개?? NO


}
