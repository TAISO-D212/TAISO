package com.d212.taiso.global.security.handler;
/**
 * Created by 전근렬 on 2024-03-21
 */

import com.google.gson.Gson;
import jakarta.servlet.ServletException;
import jakarta.servlet.http.HttpServletRequest;
import jakarta.servlet.http.HttpServletResponse;
import lombok.extern.log4j.Log4j2;
import org.springframework.security.core.AuthenticationException;
import org.springframework.security.web.authentication.AuthenticationFailureHandler;

import java.io.IOException;
import java.io.PrintWriter;
import java.util.Map;

// 성공만 있으면 되는거 아냐? 왜 실패도 만들어야 되는 것이야??
// api 요청을 했는데 서버 에러, 그런데 왜 안 되는지 우리 입장에서 모르잖아 (결제 에러인지 로그인 에러 인지 등.)
// 그래서 api 서버의 경우는 잘못됐을 때는 어떻게든 메시지를 전달을 해줘야 됨
// 200을 주면서 메시지는 로그인 실패를 보냄.
@Log4j2
public class APILoginFailHandler implements AuthenticationFailureHandler {

    @Override
    public void onAuthenticationFailure(HttpServletRequest request, HttpServletResponse response,
        AuthenticationException exception) throws IOException, ServletException {
        log.info("Login fail....." + exception);
        Gson gson = new Gson();
        String jsonStr = gson.toJson(Map.of("error", "ERROR_LOGIN"));
        response.setContentType("application/json");
        PrintWriter printWriter = response.getWriter();
        printWriter.println(jsonStr);
        printWriter.close();
    }

}