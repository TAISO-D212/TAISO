package com.d212.taiso.global.security.handler;
/**
 * Created by 전근렬 on 2024-03-21
 */

import com.d212.taiso.domain.member.dto.MemberDTO;
import com.d212.taiso.global.result.error.ErrorCode;
import com.d212.taiso.global.result.error.exception.BusinessException;
import com.d212.taiso.global.security.util.JWTUtil;
import com.google.gson.Gson;
import jakarta.servlet.ServletException;
import jakarta.servlet.http.HttpServletRequest;
import jakarta.servlet.http.HttpServletResponse;
import lombok.extern.log4j.Log4j2;
import org.springframework.security.core.Authentication;
import org.springframework.security.web.authentication.AuthenticationSuccessHandler;

import java.io.IOException;
import java.io.PrintWriter;
import java.util.Map;

// 인증에 성공했을 때 어떻게 할꺼야?
@Log4j2
public class APILoginSuccessHandler implements AuthenticationSuccessHandler {

    @Override
    public void onAuthenticationSuccess(HttpServletRequest request, HttpServletResponse response,
        Authentication authentication) throws IOException, ServletException {
        log.info("-------------------");
        log.info("authentication");
        log.info("-------------------");

        // 여기까지 왔다는 것은 인증 성공을 했다는 것
        // MemberDTO를 꺼내와가지고 JWT
        // MemberDTO는 인증한 것에서부터 getPrincipal로 부터 끄집어냄
        MemberDTO memberDTO = (MemberDTO) authentication.getPrincipal();

        // Todo 로그인 시에 deleteFlag 값이 1이면 예외처리 하기
        if (memberDTO.isDeleteFlag()) {
            throw new BusinessException(ErrorCode.MEMBER_DISABLED);
        }

        // 우리가 만들어야 될 데이터
        // 이걸 왜 이렇게 따로 끄집어냈냐면 이따가 추가적인 정보(엑세스 토큰과 리프레스 토큰)를 넣을 것이므로
        Map<String, Object> claims = memberDTO.getClaims();

        // Todo 엑세스 토큰 만들기
        String accessToken = JWTUtil.generateToken(claims, 10);
        String refreshToken = JWTUtil.generateToken(claims, 60 * 24);

        claims.put("accessToken", accessToken);
        claims.put("refreshToken", refreshToken);

        // 이렇게 claims에 정보를 넣고난 뒤에 json문자열로 만들 것이다.
        Gson gson = new Gson();

        String jsonStr = gson.toJson(claims);

        // 여기서부턴 기본적인 웹 프로그래밍이 튀어나옴(Java.io 문법)
        response.setContentType("application/json; charset=UTF-8");

        // PrintWriter는 Java에서 텍스트 데이터를 파일이나 다른 출력 대상에 쉽게 출력할 수 있도록 도와주는 클래스입니다.

        PrintWriter printWriter = response.getWriter();
        printWriter.println(jsonStr);
        printWriter.close();
    }
}
