package com.d212.taiso.domain.member.service;

import com.d212.taiso.domain.member.dto.MemberJoinReq;
import com.d212.taiso.domain.member.entity.Member;
import com.d212.taiso.domain.member.repository.MemberRepository;
import lombok.RequiredArgsConstructor;
import lombok.extern.log4j.Log4j2;
import org.springframework.security.crypto.password.PasswordEncoder;
import org.springframework.stereotype.Service;

import java.time.LocalDateTime;

@Log4j2
@Service
@RequiredArgsConstructor
public class MemberServiceImpl implements MemberService{

    private final MemberRepository memberRepository;

    private final PasswordEncoder passwordEncoder;
    @Override
    public boolean checkEmail(String email) {
        return memberRepository.existsByEmail(email);
    }

    @Override
    public void memberJoin(MemberJoinReq memberJoinReq) {
        Member member = Member.builder()
                .email(memberJoinReq.getEmail())
                .pw(passwordEncoder.encode(memberJoinReq.getPw()))
                .name(memberJoinReq.getName())
                .createTime(LocalDateTime.now())
                .faceImg(memberJoinReq.getFaceImg())
                .build();
        memberRepository.save(member);
    }
}
