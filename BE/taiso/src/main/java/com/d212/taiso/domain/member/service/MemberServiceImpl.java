package com.d212.taiso.domain.member.service;
/**
 * Created by 전근렬 on 2024-03-21
 */

import com.d212.taiso.domain.member.dto.MemberJoinReq;
import com.d212.taiso.domain.member.dto.MemberRes;
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

//    private final AlarmRepository alarmRepository; // 알림 엔티티를 DB에 저장하고 검색하기 위한 레포지토리
//    private final MemberRepository memberRepository; // 사용자 엔티티를 데이터베이스에서 검색하기 위한 레포지토리


    @Override
    public boolean checkEmail(String email) {
        return memberRepository.existsByEmail(email);
    }

    @Override
    public MemberRes getMember() {
        Member member = commonUtil.getMember();
        return MemberRes.builder()
                .email(member.getEmail())
                .name(member.getName())
                .faceImg(member.getFaceImg())
                .createDate(member.getCreateDate())
                .build();
    }

    @Override
    @Transactional
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
    public void memberDelete() {
        // 회원을 조회 후 소프트 delete 시킵니다.
        Member member = commonUtil.getMember();
        member.changeDeleteFlag(true);

    }

    @Override
    @Transactional
    public void saveFcmToken(String fcmToken) {
        Member member = commonUtil.getMember();
//        Member member = memberRepository.findMemberByEmail()
//                .orElseThrow(() -> new BusinessException(ErrorCode.MEMBER_EMAIL_NOT_EXIST));//존재하지 않는 이메일이면..

        member.changeFcmToken(fcmToken);
        memberRepository.save(member);
    }


//    @Transactional //해당 메서드가 트랜잭션 내에서 실행되어야 함을 나타냄. 메서드 내에서 발생하는 모든 데이터베이스 작업은 하나의 트랜잭션으로 묶임.
//    //알림을 저장하는 메서드 -> 토큰을 매개변수로 받아들임.
//    public void saveAlert(String token) {
//        Member member = memberRepository.findMemberByEmail(email); // 현재 로그인한 사용자의 이메일을 사용하여 사용자를 데이터베이스에서 검색
//
////        // 받은 토큰으로 새로운 알림 객체를 생성
////        Alarm alarm = Alarm.builder()
////                .token(token)
////                .build();
//
////        alarm.confirmUser(member); //   알림에 사용자를 연결시킴. 사용자와 알림 사이의 관계를 설정하는 메서드
//        memberRepository.save(alert); // 생성된 알림 객체를 데이터베이스에 저장
//    }


}
