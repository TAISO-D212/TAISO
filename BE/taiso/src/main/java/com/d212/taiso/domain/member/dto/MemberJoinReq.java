package com.d212.taiso.domain.member.dto;
/**
 * Created by 전근렬 on 2024-03-21
 */

import lombok.Data;

@Data
public class MemberJoinReq {

    private String email;
    private String pwd;
    private String name;
    private String faceImg;

}
