package com.d212.taiso.domain.reservation.repository;

import com.d212.taiso.domain.member.entity.Member;
import com.d212.taiso.domain.reservation.entity.Reservation;
import java.util.List;
import org.springframework.data.jpa.repository.JpaRepository;

/**
 * Created by 전근렬 on 2024-03-22
 */


public interface ReservationRepository extends JpaRepository<Reservation, String> {

    // 보류 Todo
    // 현재 멤버가 예약한 리스트들 -> 이거 왜 멤버로만 뜨지??, email로 하고 싶은데 -> 단방향이라 그럼
    List<Reservation> findReservationsByMember(Member member);


}

