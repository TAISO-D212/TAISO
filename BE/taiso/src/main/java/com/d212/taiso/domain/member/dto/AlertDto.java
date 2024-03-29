package com.d212.taiso.domain.member.dto;


import lombok.Builder;
import lombok.Getter;
import lombok.NoArgsConstructor;


// 메시지를 담을 DTO
@Getter
@NoArgsConstructor
public class AlertDto {

    private String title;   // 메시지 제목
    private String message; // 메시지 내용
    private String token;   // fcm 토큰

    @Builder
    public AlertDto(String title, String message, String token) {
        this.title = title;
        this.message = message;
        this.token = token;
    }
}