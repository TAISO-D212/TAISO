package com.d212.taiso.domain.reservation.repository;

import com.d212.taiso.domain.member.entity.Member;
import com.d212.taiso.domain.reservation.entity.RsvDetail;
import com.d212.taiso.domain.reservation.entity.RsvDetailId;
import java.util.List;
import java.util.Optional;
import org.springframework.data.jpa.repository.JpaRepository;

public interface RsvDetailRepository extends JpaRepository<RsvDetail, RsvDetailId> {

    Optional<RsvDetail> findRsvDetailByMemberAndRsvDetailId(Member member, RsvDetailId rsvDetailId);

    List<RsvDetail> findRsvDetailByMember(Member member);
}
