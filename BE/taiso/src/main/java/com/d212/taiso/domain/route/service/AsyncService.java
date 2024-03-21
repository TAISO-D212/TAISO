package com.d212.taiso.domain.route.service;

import com.d212.taiso.domain.route.dto.LocationDto;
import com.d212.taiso.domain.route.mqtt.Publisher;
import com.fasterxml.jackson.core.JsonProcessingException;
import com.fasterxml.jackson.databind.ObjectMapper;
import java.util.HashMap;
import java.util.Map;
import java.util.concurrent.CompletableFuture;
import lombok.extern.log4j.Log4j2;
import lombok.RequiredArgsConstructor;
import org.springframework.scheduling.annotation.Async;
import org.springframework.stereotype.Service;
import java.util.List;

/**
 * Created by 배성연 on 2024-03-21
 */

@Log4j2
@Service
@RequiredArgsConstructor
public class AsyncService {

    private final Publisher publisher;
    private final ObjectMapper objectMapper;

    @Async("taskExecutor")
    public CompletableFuture<Void> locationToRoute(List<LocationDto> locations, long rsvId,
        long placeId) {
        try {
            Map<String, Object> dataMap = new HashMap<>();
            dataMap.put("rsvId", rsvId);
            dataMap.put("placeId", placeId);
            dataMap.put("locations", locations);
            String payload = objectMapper.writeValueAsString(dataMap);

            // location 데이터로 MQTT 메시지 발행
            publisher.publishLocations("distance", payload);
            log.debug("locations로 발행 완료, rsvId : {}, payload : {}", rsvId, payload);

            // 필요한 경우 추가 작업을 수행할 수 있도록
            return CompletableFuture.completedFuture(null);
        } catch (JsonProcessingException e) {
            log.error("payload 생성 에러 {}", e);
            CompletableFuture<Void> failedFuture = new CompletableFuture<>();
            failedFuture.completeExceptionally(e);
            return failedFuture;
        }
    }

    @Async("taskExecutor")
    public void DistanceCalc(String payload) {
        try {
            Map<String, Object> responseData = objectMapper.readValue(payload, Map.class);
            log.debug("distance 수신값 : {}", responseData);

            long rsvId = (long) responseData.get("rsvId");
            long placeId = (long) responseData.get("placeId");

            // distance로 경로 계산, DB 업데이트

        } catch (JsonProcessingException e) {
            log.error("distance 처리 에러 {}", e);
        }
    }

}
