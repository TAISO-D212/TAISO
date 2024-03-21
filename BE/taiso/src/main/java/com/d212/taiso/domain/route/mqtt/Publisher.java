//package com.d212.taiso.domain.route.mqtt;
//
//import com.d212.taiso.domain.route.dto.LocationDto;
//import com.fasterxml.jackson.core.JsonProcessingException;
//import com.fasterxml.jackson.databind.ObjectMapper;
//import lombok.RequiredArgsConstructor;
//import lombok.extern.log4j.Log4j2;
//import org.springframework.messaging.support.MessageBuilder;
//import org.springframework.stereotype.Component;
//import org.springframework.messaging.MessageChannel;
//import java.util.List;
//
///**
// * Created by 배성연 on 2024-03-21
// */
//
//@Log4j2
//@Component
//@RequiredArgsConstructor
//public class Publisher {
//
//    private final MessageChannel mqttOutputChannel;
//    private final ObjectMapper objectMapper;
//
//    public void publishLocations(String topic, String payload) {
//        mqttOutputChannel.send(MessageBuilder.withPayload(payload)
//            .setHeader("mqtt_topic", topic).build());
//        log.debug("Published locations: " + payload);
//
//    }
//
//}
