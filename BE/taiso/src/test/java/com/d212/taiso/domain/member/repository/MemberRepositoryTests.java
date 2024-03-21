package com.d212.taiso.domain.member.repository;

import com.d212.taiso.domain.member.entity.Member;
import com.d212.taiso.global.exception.member.MemberNotFoundException;
import lombok.extern.log4j.Log4j2;
import org.junit.jupiter.api.Test;
import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.boot.test.context.SpringBootTest;
import org.springframework.security.crypto.password.PasswordEncoder;

import static org.junit.jupiter.api.Assertions.*;

@SpringBootTest
@Log4j2
class MemberRepositoryTests {

    @Autowired
    private MemberRepository memberRepository;

    @Autowired
    private PasswordEncoder passwordEncoder;

    @Test
    void testInsertMember() {
        for (int i = 0; i < 10; i++) {
            Member member = Member.builder()
                .email("user" + i + "@aaa.com")
                .pwd(passwordEncoder.encode("1111"))
                .name("USER" + i)
                .build();
            memberRepository.save(member);
        }
    }

    @Test
    void testFindMemberByEmail() {
        String email = "user7@aaa.com";
        Member member = memberRepository.findMemberByEmail(email)
            .orElseThrow(MemberNotFoundException::new);
        log.info("-----------------");
        log.info(member);


    }


}