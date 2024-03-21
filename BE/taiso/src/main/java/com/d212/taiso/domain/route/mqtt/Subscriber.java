package com.d212.taiso.domain.route.mqtt;
/**
 * Created by 배성연 on 2024-03-21
 */

import com.d212.taiso.domain.route.service.AsyncService;
import lombok.extern.log4j.Log4j2;
import lombok.RequiredArgsConstructor;
import org.springframework.integration.annotation.ServiceActivator;
import org.springframework.messaging.MessageHandler;
import org.springframework.messaging.MessageHeaders;
import org.springframework.stereotype.Component;

@Log4j2
@Component
@RequiredArgsConstructor
public class Subscriber {

    private final AsyncService asyncService;

    @ServiceActivator(inputChannel = "mqttInputChannel")
    public MessageHandler messageHandler() {
        return message -> {
            String payload = message.getPayload().toString();
            MessageHeaders headers = message.getHeaders();
            String topic = headers.get("mqtt_topic", String.class);

            switch (topic) {
                case "location/BE":
                    // 실시간 위치 받아왔다면 갱신

                    break;
                case "distance":
                    // 거리 계산값 받아왔다면
                    asyncService.DistanceCalc(payload);
                    break;
                default:
                    log.error(",unknown topic received! : {}", topic);
            }
        };
    }


}
