package com.d212.taiso.domain.member.repository;

import com.d212.taiso.domain.member.entity.Member;
import org.springframework.data.jpa.repository.JpaRepository;
import org.springframework.data.jpa.repository.Query;
import org.springframework.data.repository.query.Param;

public interface MemberRepository extends JpaRepository<Member, String> {

//    @Query("select m from Member m where m.email = :email")
    Member findMemberByEmail(String email);
    boolean existsByEmail(String email);

}
