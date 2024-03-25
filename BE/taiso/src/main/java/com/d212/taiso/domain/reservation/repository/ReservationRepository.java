package com.d212.taiso.domain.reservation.repository;

import com.d212.taiso.domain.member.entity.Member;
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
    // 현재 멤버가 예약한 리스트들 -> 이거 왜 멤버로만 뜨지??, email로 하고 싶은데 -> 단방향이라 그럼
    List<Reservation> findReservationsByMember(Member member);


    // 현재 시간 이후의 모든 예약 리스트 조회
    @Query("select r from Reservation r where r.time >= :currentTime")
    List<Reservation> findAllAfterCurrentTime(LocalDateTime currentTime);


    // 현재 시간대의 예약이 있는지 조회
    @Query("select r from Reservation r where r.time = :currentTime")
    Optional<Reservation> findCurrentTime(LocalDateTime currentTime);

}

