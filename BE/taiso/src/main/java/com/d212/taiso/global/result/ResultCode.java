package com.d212.taiso.global.result;
/**
 * Created by 전근렬 on 2024-03-21
 */

import lombok.AllArgsConstructor;
import lombok.Getter;

@Getter
@AllArgsConstructor
public enum ResultCode {

    // Member
    JOIN_SUCCESS(200, "회원가입에 성공하였습니다."),
    LOGIN_SUCCESS(200, "로그인에 성공하였습니다."),
    EMAIL_DO_EXIST(200, "이미 존재하는 이메일 입니다."),
    EMAIL_DO_NOT_EXIST(200, "사용 가능한 이메일 입니다."),
    GET_MEMBER_SUCCESS(200, "회원 조회에 성공하였습니다."),
    MEMBER_DELETE_SUCCESS(200, "회원 탈퇴에 성공하였습니다."),

    //Bookmark
    GET_BOOKMARK_SUCCESS(200, "즐겨찾기 조회에 성공하였습니다."),
    ADD_BOOKMARK_SUCCESS(200, "즐겨찾기 생성에 성공하였습니다."),
    DELETE_BOOKMARK_SUCCESS(200, "즐겨찾기 삭제에 성공하였습니다."),

    // Reservation
    GET_RESERVATION_SUCCESS(200, "예약 목록 조회에 성공하였습니다."),
    GET_MY_RESERVATION_SUCCESS(200, "내 예약 목록 조회에 성공하였습니다."),
    ADD_RESERVATION_SUCCESS(200, "예약 생성에 성공하였습니다."),
    ADD_RESERVATION_FAIL(400, "예약 생성에 실패하였습니다."),
    ADD_TOGETHER_RESERVATION_SUCCESS(200, "합승 예약 생성에 성공하였습니다."),
    ADD_TOGETHER_RESERVATION_FAIL(200, "합승 예약 생성에 실패하였습니다."),
    DELETE_RESERVATION_SUCCESS(200, "예약 삭제에 성공하였습니다.");

    private final int status;
    private final String message;
}
