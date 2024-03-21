package com.d212.taiso.domain.route.mqtt;

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
        mqttOutputChannel.send(MessageBuilder.withPayload(payload)
            .setHeader("mqtt_topic", topic).build());
        log.debug("Published locations: " + payload);

    }

}
