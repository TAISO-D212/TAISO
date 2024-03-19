package com.d212.taiso.domain.member.service;

import com.d212.taiso.domain.member.dto.MemberJoinReq;
import com.d212.taiso.domain.member.entity.Member;

public interface MemberService {

    boolean checkEmail(String email);

    void memberJoin(MemberJoinReq memberJoinReq);

    // 비밀번호 암호화로 인해 지움.
//    default Member memberJoinReqToEntity(MemberJoinReq memberJoinReq){
//        return Member.builder()
//                .email(memberJoinReq.getEmail())
//                .pw(memberJoinReq.getPw())
//                .name(memberJoinReq.getName())
//                .faceImg(memberJoinReq.getFaceImg())
//                .build();
//    }
}
