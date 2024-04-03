package com.d212.taiso.domain.member.repository;
/**
 * Created by 전근렬 on 2024-03-21
 */

import com.d212.taiso.domain.member.entity.Member;
import org.springframework.data.jpa.repository.JpaRepository;
import org.springframework.data.jpa.repository.Query;
import org.springframework.data.repository.query.Param;

import java.util.Optional;

public interface MemberRepository extends JpaRepository<Member, String> {


//        @Query("select m from Member m where m.email = :email")
    Optional<Member> findMemberByEmail(String email);

    boolean existsByEmail(String email);

    @Query("SELECT r.member FROM Reservation r WHERE r.id = :rsvId")
    Member findMemberByRsvId(Long rsvId);

}
