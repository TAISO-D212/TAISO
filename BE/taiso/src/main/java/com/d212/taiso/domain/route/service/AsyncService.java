//package com.d212.taiso.domain.route.service;
//
//import com.d212.taiso.domain.route.dto.LocationDto;
//import com.d212.taiso.domain.route.mqtt.Publisher;
//import com.fasterxml.jackson.core.type.TypeReference;
//import com.fasterxml.jackson.databind.ObjectMapper;
//import java.util.HashMap;
//import java.util.Map;
//import java.util.concurrent.CompletableFuture;
//import java.util.concurrent.Executors;
//import lombok.extern.log4j.Log4j2;
//import lombok.RequiredArgsConstructor;
//import org.springframework.scheduling.annotation.Async;
//import org.springframework.stereotype.Service;
//import java.util.List;
//
///**
// * Created by 배성연 on 2024-03-21
// */
//
//@Log4j2
//@Service
//@RequiredArgsConstructor
//public class AsyncService {
//
//    private final Publisher publisher;
//    private final ObjectMapper objectMapper;
//
//    @Async("taskExecutor")
//    public CompletableFuture<Void> locationToRoute(List<LocationDto> locations, long rsvId) {
//        try {
//            Map<String, Object> dataMap = new HashMap<>();
//            dataMap.put("rsvId", rsvId);
//            dataMap.put("locations", locations);
//            String payload = objectMapper.writeValueAsString(dataMap);
//
//            // location 데이터로 MQTT 메시지 발행
//            publisher.publishLocations("distance", payload);
//
//            // CompletableFuture 생성으로 메시지 응답 대기 및 처리
//            return CompletableFuture.runAsync(() -> {
//                // "distance" 토픽  대기 및 응답 처리
//                // 이 부분은 실제 응답을 어떻게 대기할 것인지에 따라 다르게 구현
//                // 예를 들어, MQTT 클라이언트의 콜백 메커니즘을 사용하거나,
//                // 메시지 대기를 위한 별도의 로직이 필요할 수 있습니다.
////            try {
////                List<Integer> distances = objectMapper.readValue(payload,
////                    new TypeReference<List<Integer>>() {
////                    });
////                log.debug("받은 거리 배열 : {}", distances);
////                // distances 리스트 처리
////
////            } catch (Exception e) {
////                log.error("거리 받아와서 처리 중 오류 발생 : {}", e.getMessage(), e);
////            }
//
//            }, Executors.newCachedThreadPool());    // 별도의 스레드 풀에서 진행
//
//        } catch (Exception e) {
//
//        }
//    }
//
//}
