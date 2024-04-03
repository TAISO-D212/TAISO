package com.d212.taiso.domain.member.service;
/**
 * Created by 전근렬 on 2024-03-21
 */

import com.d212.taiso.domain.member.dto.MemberJoinReq;
import com.d212.taiso.domain.member.dto.MemberRes;
import org.springframework.transaction.annotation.Transactional;

import java.util.List;

public interface MemberService {

    boolean checkEmail(String email);

    MemberRes getMemberInfo();

    void memberJoin(MemberJoinReq memberJoinReq);

    void memberDelete();

    @Transactional
    void saveFcmToken(String fcmToken);

    List<String> getFcmToken(Long rsvId);

    // 비밀번호 암호화로 인해 지움.
//    default Member memberJoinReqToEntity(MemberJoinReq memberJoinReq){
//        return Member.builder()
//                .email(memberJoinReq.getEmail())
//                .pwd(memberJoinReq.getPwd())
//                .name(memberJoinReq.getName())
//                .faceImg(memberJoinReq.getFaceImg())
//                .build();
//    }
}
