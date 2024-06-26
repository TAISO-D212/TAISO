package com.d212.taiso.global.result.error;
/**
 * Created by 전근렬 on 2024-03-21
 */

import lombok.AllArgsConstructor;
import lombok.Getter;

@Getter
@AllArgsConstructor
public enum ErrorCode {
    // Global
    INTERNAL_SERVER_ERROR(500, "내부 서버 오류입니다."),
    METHOD_NOT_ALLOWED(405, "허용되지 않은 HTTP method입니다."),
    INPUT_VALUE_INVALID(400, "유효하지 않은 입력입니다."),
    INPUT_TYPE_INVALID(400, "입력 타입이 유효하지 않습니다."),
    HTTP_MESSAGE_NOT_READABLE(400, "request message body가 없거나, 값 타입이 올바르지 않습니다."),
    HTTP_HEADER_INVALID(400, "request header가 유효하지 않습니다."),
    ENTITY_NOT_FOUNT(500, "존재하지 않는 Entity입니다."),
    FORBIDDEN_ERROR(403, "작업을 수행하기 위한 권한이 없습니다."),
    IS_NOT_IMAGE(400, "이미지가 아닙니다."),

    //JWT
    JWT_INVALID(401, "유효하지 않은 토큰입니다."),
    JWT_BADTYPE(401, "Bearer 타입 토큰이 아닙니다."),
    JWT_EXPIRED(403, "만료된 토큰입니다."),
    JWT_MALFORM(401, "토큰값이 올바르지 않습니다."),
    BLACK_TOKEN(401, "접근이 차단된 토큰입니다."),
    TOKEN_ALIVE(400, "유효기간이 만료되지 않은 토큰입니다."),
    REFRESH_INVALID(400, "리프레시 토큰이 유효하지 않습니다."),

    //Member
    MEMBER_EMAIL_NOT_EXIST(400, "현재 email에 해당하는 회원이 존재하지 않습니다."),
    MEMBER_EMAIL_DUPLICATED(400, "이미 존재하는 이메일입니다."),
    MEMBER_DISABLED(400, "현재 비활성화 된 회원입니다."),

    //Place
    PLACE_ID_NOT_EXIST(400, "현재 placeId에 해당하는 장소가 존재하지 않습니다."),

    //Bookmark
    BOOKMARK_NOT_EXIST(400, "즐겨찾기가 존재하지 않습니다."),

    // Reservation
    RESERVATION_NOT_EXIST(400, "예약이 존재하지 않습니다."),

    //RsvDetail
    RSV_DETAIL_NOT_EXIST(400, "예약 상세내용이 존재하지 않습니다.");


    private final int status;
    private final String message;
}
