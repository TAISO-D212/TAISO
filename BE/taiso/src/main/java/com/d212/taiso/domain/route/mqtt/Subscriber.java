package com.d212.taiso.domain.route.mqtt;
/**
 * Created by 배성연 on 2024-03-21
 */

import com.d212.taiso.domain.route.service.AsyncService;
import java.nio.charset.StandardCharsets;
import lombok.extern.log4j.Log4j2;
import lombok.RequiredArgsConstructor;
import org.springframework.context.annotation.Bean;
import org.springframework.context.annotation.Configuration;
import org.springframework.integration.annotation.ServiceActivator;
import org.springframework.messaging.MessageHandler;
import org.springframework.messaging.MessageHeaders;
import org.springframework.stereotype.Component;

@Log4j2
@Configuration
@RequiredArgsConstructor
public class Subscriber {

    private final AsyncService asyncService;

    @Bean
    @ServiceActivator(inputChannel = "mqttInputChannel")
    public MessageHandler messageHandler() {
        return message -> {
            log.info("메시지 수신! Subscriber 실행");
            Object payload = message.getPayload();
            MessageHeaders headers = message.getHeaders();
            String topic = headers.get("mqtt_receivedTopic", String.class);
            log.info("topic : {} , payload : {}", topic, payload);

            switch (topic) {
                case "location/BE":
                    // 실시간 위치 받아왔다면 갱신
                    log.info("실시간 위치 입력 task 실행");
                    asyncService.saveLocation(payload.toString());
                    break;
                case "distance/EMB":
                    // 거리 계산값 받아왔다면
                    log.info("거리 계산 task 실행");
                    asyncService.calcDistance(payload.toString());
                    break;
                default:
                    log.error(",unknown topic received! : {}", topic);
            }
        };
    }


}
