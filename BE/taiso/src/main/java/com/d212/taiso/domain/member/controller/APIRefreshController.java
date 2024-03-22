package com.d212.taiso.domain.member.controller;
/**
 * Created by 전근렬 on 2024-03-21
 */
// Controller 말고 필터로 만들 수도 있는데
// 제이슨 처리도 귀찮고 이미 한번 해봤으므로 Controller ㄱ

// 엑세스 토큰은 Authorization라는 헤더로 오고
// 리프레쉬 토큰은 직접 와야 함

// 로그인 part는 config에서 확인하슈

import com.d212.taiso.global.security.util.CustomJWTException;
import com.d212.taiso.global.security.util.JWTUtil;
import lombok.RequiredArgsConstructor;
import lombok.extern.log4j.Log4j2;
import org.springframework.web.bind.annotation.RequestHeader;
import org.springframework.web.bind.annotation.RequestMapping;
import org.springframework.web.bind.annotation.RestController;

import java.util.Map;

@RestController
@RequiredArgsConstructor
@Log4j2
public class APIRefreshController {

    @RequestMapping("/members/refresh")
    public Map<String, Object> refresh(
        @RequestHeader("Authorization") String authHeader,
        String refreshToken
    ) {

        if (refreshToken == null) {
            throw new CustomJWTException("NULL_REFRESH");
        }

        if (authHeader == null || authHeader.length() < 7) {
            throw new CustomJWTException("INVALID STRING");
        }

        // Bearer xxxx
        String accessToken = authHeader.substring(7);

        // AccessToken 만료여부 확인
        //Access 토큰이 만료되지 않았다면 그냥 같은걸로...
        if (checkExpiredToken(accessToken) == false) {
            return Map.of("accessToken", accessToken, "refreshToken", refreshToken);
        }

        //Refresh토큰 검증
        Map<String, Object> claims = JWTUtil.validateToken(refreshToken);
        log.info("refresh ... claims: " + claims);
        String newAccessToken = JWTUtil.generateToken(claims, 10 * 24 * 60);
        // Todo 여기도 시간 체크
        String newRefreshToken = checkTime((Integer) claims.get("exp")) == true ?
            JWTUtil.generateToken(claims, 60 * 24) : refreshToken;
        return Map.of("accessToken", newAccessToken, "refreshToken", newRefreshToken);
    }

    //시간이 1시간 미만으로 남았다면
    private boolean checkTime(Integer exp) {
        //JWT exp를 날짜로 변환
        java.util.Date expDate = new java.util.Date((long) exp * (1000));
        //현재 시간과의 차이 계산 - 밀리세컨즈
        long gap = expDate.getTime() - System.currentTimeMillis();
        //분단위 계산
        long leftMin = gap / (1000 * 60);
        //1시간도 안남았는지..
        return leftMin < 60;
    }

    private boolean checkExpiredToken(String token) {
        try {
            JWTUtil.validateToken(token);
        } catch (CustomJWTException ex) {
            if (ex.getMessage().equals("Expired")) {
                return true;
            }
        }
        return false;
    }

}