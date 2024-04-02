package com.d212.taiso.domain.member.service;

import com.d212.taiso.domain.member.dto.AlertDto;
import com.d212.taiso.domain.member.entity.Member;
import com.d212.taiso.global.util.CommonUtil;
import com.google.firebase.messaging.FirebaseMessaging;
import com.google.firebase.messaging.Message;
import com.google.firebase.messaging.WebpushConfig;
import com.google.firebase.messaging.WebpushNotification;
import lombok.RequiredArgsConstructor;
import lombok.extern.slf4j.Slf4j;
import org.hibernate.sql.exec.ExecutionException;
import org.springframework.stereotype.Component;

@Component
@RequiredArgsConstructor
@Slf4j
public class AlertServiceImpl {

    private final MemberService memberService;
    private final CommonUtil commonUtil;
    public void arrivalAlertSend() throws ExecutionException, InterruptedException {

        AlertDto alertDto = AlertDto.builder()
                .title("TAISO")
                .token(memberService.getfcmToken())
                .message("버스가 곧 도착합니다.")
                .build();
        sendAlert(alertDto);
        log.info("보내졌습니다!");
    }


    public String getNotificationToken() {
//        Member member = commonUtil.getMember();
//        return member.getFcmToken();
       return memberService.getfcmToken();
    }


    public void sendAlert(AlertDto alert) throws ExecutionException, InterruptedException {
        Message message = Message.builder()
                .setWebpushConfig(WebpushConfig.builder()
                        .setNotification(WebpushNotification.builder()
                                .setTitle(alert.getTitle())
                                .setBody(alert.getMessage())
                                .build())
                        .build())
                .setToken(memberService.getfcmToken())
                .build();


        String response = null;
        try {
            response = FirebaseMessaging.getInstance().sendAsync(message).get();
        } catch (java.util.concurrent.ExecutionException e) {
            throw new RuntimeException(e);
        }
        log.info(">>>>Send message : " + response);
    }

}