package com.d212.taiso.global.exception.member;

public class MemberDisabledException extends RuntimeException {

    public MemberDisabledException() {
        super("해당 유저는 비활성화 된 유저입니다.");
    }
}
