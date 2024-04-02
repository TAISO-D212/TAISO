package com.d212.taiso.domain.reservation.repository;

import com.d212.taiso.domain.member.entity.Member;
import com.d212.taiso.domain.reservation.dto.MyRsvListRes;
import com.d212.taiso.domain.reservation.dto.OriginRouteInfoDto;
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

    // 현재 시간 이후의 모든 예약 리스트 조회
    @Query("select r from Reservation r where r.time >= :currentTime order by r.time")
    List<Reservation> findAllAfterCurrentTime(LocalDateTime currentTime);


    // 현재 시간대의 예약이 있는지 조회
    @Query("select r from Reservation r where r.time = :currentTime")
    Optional<Reservation> findCurrentTime(LocalDateTime currentTime);

    @Query("select new com.d212.taiso.domain.reservation.dto.OriginRouteInfoDto(r.routeDist, r.stopCnt) from Reservation r where r.id = :rsvId")
    Optional<OriginRouteInfoDto> findRouteDistAndStopCntByRsvId(Long rsvId);

    @Query("select r.place.id from Reservation r where r.id = :rsvId")
    Long findPlaceIdByReservationId(Long rsvId);

}

