package com.d212.taiso.domain.member.service;
/**
 * Created by 전근렬 on 2024-03-21
 */

import com.d212.taiso.domain.member.dto.MemberJoinReq;
import com.d212.taiso.domain.member.entity.Member;
import com.d212.taiso.domain.member.repository.MemberRepository;
import com.d212.taiso.global.result.error.ErrorCode;
import com.d212.taiso.global.result.error.exception.BusinessException;
import com.d212.taiso.global.util.CommonUtil;
import lombok.RequiredArgsConstructor;
import lombok.extern.log4j.Log4j2;
import org.springframework.security.crypto.password.PasswordEncoder;
import org.springframework.stereotype.Service;
import org.springframework.transaction.annotation.Transactional;

import java.time.LocalDateTime;

@Log4j2
@Service
@RequiredArgsConstructor
public class MemberServiceImpl implements MemberService {

    private final MemberRepository memberRepository;

    private final PasswordEncoder passwordEncoder;

    private final CommonUtil commonUtil;

    @Override
    public boolean checkEmail(String email) {
        return memberRepository.existsByEmail(email);
    }

    @Override
    public void memberJoin(MemberJoinReq memberJoinReq) {

        // 이미 존재하는 이메일이면
        if (checkEmail(memberJoinReq.getEmail())) {
            throw new BusinessException(ErrorCode.MEMBER_EMAIL_DUPLICATED);
        }

        Member member = Member.builder()
            .email(memberJoinReq.getEmail())
            .pwd(passwordEncoder.encode(memberJoinReq.getPwd()))
            .name(memberJoinReq.getName())
            .createDate(LocalDateTime.now())
            .faceImg(memberJoinReq.getFaceImg())
            .build();
        memberRepository.save(member);
    }

    @Override
    @Transactional
    public void memberDelete() {
        // 회원을 조회 후 소프트 delete 시킵니다.
        Member member = commonUtil.getMember();
        member.changeDeleteFlag(true);

    }
}
