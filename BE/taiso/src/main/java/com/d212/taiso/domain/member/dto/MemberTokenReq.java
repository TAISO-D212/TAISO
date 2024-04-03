package com.d212.taiso.domain.member.dto;

import lombok.Data;

/**
 * Created by 전근렬 on 2024-04-03
 */
@Data
public class MemberTokenReq {

    private String email;

    // FcmToken
    private String token;
}
