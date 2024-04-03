package com.d212.taiso.domain.member.service;

import com.d212.taiso.domain.member.dto.AlertDto;
import com.d212.taiso.global.util.CommonUtil;
import com.google.firebase.messaging.FirebaseMessaging;
import com.google.firebase.messaging.Message;
import com.google.firebase.messaging.WebpushConfig;
import com.google.firebase.messaging.WebpushNotification;
import lombok.RequiredArgsConstructor;
import lombok.extern.slf4j.Slf4j;
import org.hibernate.sql.exec.ExecutionException;
import org.springframework.stereotype.Service;

import java.util.List;

@Service
@RequiredArgsConstructor
@Slf4j
public class AlertServiceImpl implements AlertService {

    private final MemberService memberService;
    private final CommonUtil commonUtil;

    public void departAlertSend(Long rsvId) throws ExecutionException, InterruptedException {

        // fcm토큰 리스트를 다 가져오고
        // fcm 토큰 for문으로 돌리면서 fcm을 통해 멤버를 조회해서
        List<String> tokens = memberService.getFcmToken(rsvId);

        // 각 토큰에 대해 Alert를 보냅니다.
        for (String token : tokens) {
            AlertDto alertDto = AlertDto.builder()
                .title("depart")
                .token(token)
                .message("택시가 출발했습니다!")
                .build();
            sendAlert(alertDto);
            log.info("출발 알림 전송 완료");
        }
    }

    public void soonAlertSend(Long rsvId) throws ExecutionException, InterruptedException {

        // fcm토큰 리스트를 다 가져오고
        // fcm 토큰 for문으로 돌리면서 fcm을 통해 멤버를 조회해서
        List<String> tokens = memberService.getFcmToken(rsvId);

        // 각 토큰에 대해 Alert를 보냅니다.
        for (String token : tokens) {
            AlertDto alertDto = AlertDto.builder()
                .title("soon")
                .token(token)
                .message("택시가 곧 도착합니다!")
                .build();
            sendAlert(alertDto);
            log.info("도착 예정 알림 전송 완료");
        }
    }

    public void arrivalAlertSend(Long rsvId) throws ExecutionException, InterruptedException {

        // fcm토큰 리스트를 다 가져오고
        // fcm 토큰 for문으로 돌리면서 fcm을 통해 멤버를 조회해서
        List<String> tokens = memberService.getFcmToken(rsvId);

        // 각 토큰에 대해 Alert를 보냅니다.
        for (String token : tokens) {
            AlertDto alertDto = AlertDto.builder()
                .title("arrival")
                .token(token)
                .message("택시가 도착했습니다!")
                .build();
            sendAlert(alertDto);
            log.info("버스 도착 알림 전송 완료");
        }
    }

//    public String getNotificationToken() {
////        Member member = commonUtil.getMember();
////        return member.getFcmToken();
//       return memberService.getfcmToken(rsvId);
//    }


    public void sendAlert(AlertDto alert) throws ExecutionException, InterruptedException {
        Message message = Message.builder()
            .setWebpushConfig(WebpushConfig.builder()
                .setNotification(WebpushNotification.builder()
                    .setTitle(alert.getTitle())
                    .setBody(alert.getMessage())
                    .build())
                .build())
            .setToken(alert.getToken())
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