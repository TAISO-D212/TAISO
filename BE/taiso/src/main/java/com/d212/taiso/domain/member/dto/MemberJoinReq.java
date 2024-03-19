package com.d212.taiso.domain.member.dto;

import lombok.Data;

@Data
public class MemberJoinReq {

    private String email;
    private String pw;
    private String name;
    private String faceImg;

}
