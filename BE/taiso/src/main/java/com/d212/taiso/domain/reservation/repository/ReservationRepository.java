package com.d212.taiso.domain.reservation.repository;

import com.d212.taiso.domain.member.entity.Member;
import com.d212.taiso.domain.reservation.dto.MyRsvListRes;
import com.d212.taiso.domain.reservation.entity.Reservation;

import java.time.LocalDateTime;
import java.util.List;
import java.util.Optional;
import org.springframework.data.jpa.repository.JpaRepository;
import org.springframework.data.jpa.repository.Query;

/**
 * Created by 전근렬 on 2024-03-22
 */


public interface ReservationRepository extends JpaRepository<Reservation, Long> {

    // 보류 Todo
    // join을 해서 rsvDetail에 있는
    // placeID, latitude, longtitude, address (출발지) 를 가져와야 함
    // reservation의 placeID, latitude, longtitude, address (도착지), time, arrivalTime, cnt를 가져와야 함

    // r :reservation, d : rsvDetail

    // 생각해보니깐 그냥 디테일에서 유저 있는 부분만 뽑아내면 이 짓 안해도 되는 것 아닌가?
    // 잘 만지면 이걸로 조회해서 코드 양을 줄일 수 있겠지만.. 현재 내 능력 부족
    // 차후 수정 예정
//    @Query("select "
//        + "new com.d212.taiso.domain.reservation.dto.MyRsvListRes(r.id, d.place.id, d.place.latitude, d.place.longitude, d.place.address, r.place.id, r.place.latitude, r.place.longitude, r.place.address, r.arrivalTime, r.cnt) "
//        + "from "
//        + "Reservation r left join RsvDetail d on d.rsvDetailId.reservation = r "
//        + "where "
//        + "d.member = :member")
//    List<MyRsvListRes> findMyReservationList(Member member);


    // 현재 시간 이후의 모든 예약 리스트 조회
    @Query("select r from Reservation r where r.time >= :currentTime")
    List<Reservation> findAllAfterCurrentTime(LocalDateTime currentTime);


    // 현재 시간대의 예약이 있는지 조회
    @Query("select r from Reservation r where r.time = :currentTime")
    Optional<Reservation> findCurrentTime(LocalDateTime currentTime);

}

