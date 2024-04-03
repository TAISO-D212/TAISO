package com.d212.taiso.global.security.filter;
/**
 * Created by 전근렬 on 2024-03-21
 */

import com.d212.taiso.domain.member.dto.MemberDTO;
import com.d212.taiso.global.security.util.JWTUtil;
import com.google.gson.Gson;
import jakarta.servlet.FilterChain;
import jakarta.servlet.ServletException;
import jakarta.servlet.http.HttpServletRequest;
import jakarta.servlet.http.HttpServletResponse;
import lombok.extern.log4j.Log4j2;
import org.springframework.security.authentication.UsernamePasswordAuthenticationToken;
import org.springframework.security.core.context.SecurityContextHolder;
import org.springframework.web.filter.OncePerRequestFilter;

import java.io.IOException;
import java.io.PrintWriter;
import java.time.LocalDateTime;
import java.util.List;
import java.util.Map;

@Log4j2
public class JWTCheckFilter extends OncePerRequestFilter {

    // 어떤 경로로 들어오면 필터링을 해야된다. 그런데 어떤 경로는 안해도 된다.
    // (ex. 로그인 경로는 지금 로그인을 하는 것이기 때문에 JWT 토큰이 없다)
    // 이런 것들을 빼주는 역할을 하는 것이 shouldNotFilter
    @Override
    protected boolean shouldNotFilter(HttpServletRequest request) throws ServletException {

        // true == not checking

        String path = request.getRequestURI();

        log.info("check uri----------" + path);

        // 회원 쪽은 체크하지 마!

        // 중복 체크 관련
        if (path.startsWith("/api/members/ck")) {
            return true;
        }

        if (path.startsWith("/api/members/join")) {
            return true;
        }

        if (path.startsWith("/api/members/login")) {
            return true;
        }

        if (path.startsWith("/api/members/new")) {
            return true;
        }

        // SWAGGER쪽
        if (path.startsWith("/api/taiso-ui.html")) {
            return true;
        }

        if (path.startsWith("/api/api-docs")) {
            return true;
        }

        if (path.startsWith("/api/v3/api-docs")) {
            return true;
        }

        if (path.startsWith("/api/swagger-ui")) {
            return true;
        }

        if (path.startsWith("/api/swagger-resources")) {
            return true;
        }

        //이미지 조회 경로는 체크하지 않음
//        if(path.startsWith("/api/products/view/")) {
//            return true;
//        }

        // 부정의 부정은 긍정, 즉 false == check
        return false;
    }

    @Override
    protected void doFilterInternal(HttpServletRequest request, HttpServletResponse response,
        FilterChain filterChain) throws ServletException, IOException {
        log.info("-------------------");
        log.info("-----------------JWTCheckFilter.................");
        log.info("-------------------");

        String authHeaderStr = request.getHeader("Authorization");
        //Bearer //7 JWT 문자열 , Bearer과 공백 포함해서 총 7자 뒤에 JWT 문자열이 오므로 잘라내는 작업 필요.

        try {
            //Bearer accestoken...
            String accessToken = authHeaderStr.substring(7);
            Map<String, Object> claims = JWTUtil.validateToken(accessToken);
            log.info("JWT claims: " + claims);

            // 권한 관련-------------------------(참고)
            // 사용자의 토큰이 성공을 했다면 사용자에 대한 정보를 끄집어낼 수 o
            // 이 사용자의 정보를 가지고 멤버 DTO를 구성
            String email = (String) claims.get("email");
            String pw = (String) claims.get("pwd");
            String nickname = (String) claims.get("nickname");
            Boolean deleteFlag = (Boolean) claims.get("deleteFlag");
            String fcmToken = (String) claims.get("fcmToken");

            MemberDTO memberDTO = new MemberDTO(email, pw, nickname, deleteFlag.booleanValue(),
                fcmToken);
            log.info("-----------------------------------");
            log.info(memberDTO);
            log.info(memberDTO.getAuthorities());
            UsernamePasswordAuthenticationToken authenticationToken
                = new UsernamePasswordAuthenticationToken(memberDTO, pw,
                memberDTO.getAuthorities());
            SecurityContextHolder.getContext().setAuthentication(authenticationToken);

            filterChain.doFilter(request, response);

        } catch (Exception e) {
            log.error("JWT Check Error..............");
            log.error(e.getMessage());

            // 문제가 생겼으면 이야기 해주는 부분.
            Gson gson = new Gson();
            String msg = gson.toJson(Map.of("error", "ERROR_ACCESS_TOKEN"));
            response.setContentType("application/json");
            PrintWriter printWriter = response.getWriter();
            printWriter.println(msg);
            printWriter.close();
        }
    }
}
