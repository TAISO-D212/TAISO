package com.d212.taiso.domain.route.mqtt;

import java.nio.charset.StandardCharsets;
import lombok.RequiredArgsConstructor;
import lombok.extern.log4j.Log4j2;
import org.springframework.messaging.support.MessageBuilder;
import org.springframework.stereotype.Component;
import org.springframework.messaging.MessageChannel;

/**
 * Created by 배성연 on 2024-03-21
 */

@Log4j2
@Component
@RequiredArgsConstructor
public class Publisher {

    private final MessageChannel mqttOutputChannel;

    public void publishLocations(String topic, String payload) {
        try {
            log.info("{} publish : {}", topic, payload);

            byte[] bytePayload = payload.getBytes(StandardCharsets.UTF_8);
            mqttOutputChannel.send(MessageBuilder.withPayload(bytePayload)
                .setHeader("mqtt_topic", topic).build());
        } catch (Exception e) {
            log.error("publish 중 에러 발생 {}", e);
        }
    }

}
