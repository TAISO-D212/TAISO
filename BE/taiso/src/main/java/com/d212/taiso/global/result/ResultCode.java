package com.d212.taiso.global.result;

import lombok.AllArgsConstructor;
import lombok.Getter;

@Getter
@AllArgsConstructor
public enum ResultCode {

    // Member
    JOIN_SUCCESS(200, "회원가입에 성공하였습니다."),
    LOGIN_SUCCESS(200, "로그인에 성공하였습니다."),
    EMAIL_DO_EXIST(200, "이미 존재하는 이메일 입니다."),
    EMAIL_DO_NOT_EXIST(200, "사용 가능한 닉네임 입니다.");
    private final int status;
    private final String message;
    }
