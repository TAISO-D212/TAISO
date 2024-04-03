package com.d212.taiso.domain.reservation.repository;

import com.d212.taiso.domain.member.entity.Member;
import com.d212.taiso.domain.reservation.entity.RsvDetail;
import com.d212.taiso.domain.reservation.entity.RsvDetailId;

import jakarta.transaction.Transactional;
import java.time.LocalDateTime;
import java.util.List;
import java.util.Optional;
import org.springframework.data.jpa.repository.JpaRepository;
import org.springframework.data.jpa.repository.Modifying;
import org.springframework.data.jpa.repository.Query;

public interface RsvDetailRepository extends JpaRepository<RsvDetail, RsvDetailId> {

    Optional<RsvDetail> findRsvDetailByMemberAndRsvDetailId(Member member, RsvDetailId rsvDetailId);

    List<RsvDetail> findRsvDetailsByMemberOrderByArrivalTime(Member member);


    // 해당 시간대에 유저가 예약한 것이 있는가??
    @Query(value = "SELECT rd FROM RsvDetail rd " +
        "WHERE rd.member = :member " +
        "AND rd.rsvDetailId.reservation.time = :time")
    Optional<RsvDetail> findRsvDetailByMemberAndTime(Member member, LocalDateTime time);

    List<RsvDetail> findRsvDetailByMember(Member member);

    // 순서대로 가져오되 rd의 순서가 아직 정해지지 않았다면(새로 추가된 경유지라면) 맨 마지막에 가져오기
    @Query(value = "SELECT * FROM rsv_detail  WHERE rsv_id = :rsvId "
        + "ORDER BY CASE WHEN orders = 0 THEN 999999 ELSE orders END", nativeQuery = true)
    List<RsvDetail> findRdByRsvId(Long rsvId);

    // 추가 순서대로 가져오기
    @Query(value = "SELECT * FROM rsv_detail  WHERE rsv_id = :rsvId "
        + "ORDER BY arrival_time", nativeQuery = true)
    List<RsvDetail> findRdByRsvIdOrderByArrivalTime(Long rsvId);
    
}
