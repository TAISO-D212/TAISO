package com.d212.taiso.global.exception.member;

public class MemberNotFoundException extends RuntimeException {

    public MemberNotFoundException() {
        super("해당 유저는 존재하지 않습니다.");
    }
}
