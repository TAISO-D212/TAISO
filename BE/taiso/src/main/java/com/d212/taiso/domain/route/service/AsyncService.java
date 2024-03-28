package com.d212.taiso.domain.route.service;

import com.d212.taiso.domain.reservation.dto.OriginRouteInfoDto;
import com.d212.taiso.domain.reservation.repository.ReservationRepository;
import com.d212.taiso.domain.route.dto.LocationDto;
import com.d212.taiso.domain.route.dto.LocationPayload;
import com.d212.taiso.domain.route.mqtt.Publisher;
import com.fasterxml.jackson.core.JsonProcessingException;
import com.fasterxml.jackson.databind.ObjectMapper;
import jakarta.persistence.EntityNotFoundException;
import java.util.HashMap;
import java.util.Map;
import java.util.StringTokenizer;
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
    private final ReservationRepository reservationRepository;

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
            publisher.publishLocations("distance/BE", payload);
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
    public void calcDistance(String payload) {
        // 데이터 처리 로직
        try {
            LocationPayload locationPayload = objectMapper.readValue(payload,
                LocationPayload.class);
            long rsvId = locationPayload.getRsvId();
            long placeId = locationPayload.getPlaceId();
            int re = locationPayload.getDistanceList().size();
            long[] distanceList = new long[re];

            for (int i = 0; i < re; i++) {
                distanceList[i] = locationPayload.getDistanceList().get(i);
            }
            log.info("calcDistance 호출 : rsvId {}, placeId {}, distanceList {}", rsvId, placeId,
                distanceList);
            // DB reservation table에서 경유지 수, 경유지 간 거리 받아오기
            // Service 순환 문제 해결 위해 직접 repository 접근
            OriginRouteInfoDto originRouteInfo = reservationRepository.findRouteDistAndStopCntByRsvId(
                    rsvId)
                .orElseThrow(() -> new EntityNotFoundException("예약 없음"));
            int stopCnt = originRouteInfo.getStopCnt() + 2;
            String originDistStr = originRouteInfo.getRouteDist();

            int[][] distTable;
            int lastNum = stopCnt + 1;
            if (originDistStr == null) {
                // 신규 예약 시 -> [새 경유지와 주차장, 목적지, 주차장-목적지 거리]
                //          주차장     목적지     새 경유지
                // 주차장
                // 목적지
                // 새 경유지
                lastNum = 3;
                distTable = new int[lastNum][lastNum];

            } else {
                // 경유지 추가 시 -> [새 경유지와 주차장, 목적지, 경유지1까지 각각 거리]
                // 받아온 경유지 간 거리를 테이블로 만들기
                //          주차장     목적지     경유지 1   새 경유지
                // 주차장       0       500        400
                // 목적지      500       0         300
                // 경유지 1    400      300         0
                // 새 경유지                                 0
                lastNum++;
                distTable = new int[lastNum][lastNum];
                StringTokenizer stk = new StringTokenizer(originDistStr);
                for (int i = 0; i < stopCnt; i++) {
                    for (int j = 0; j < stopCnt; j++) {
                        distTable[i][j] = Integer.parseInt(stk.nextToken());
                    }
                }
            }

            // distance로 다익스트라 경로 계산

            // 최소 거리 총합이 30km 넘으면 예약 삭제 DB 업데이트, 예약 불가 push 알림

            // 최소 거리 총합이 30km 이내면 예약 성공 DB 업데이트, 예약 성공 push 알림
        } catch (JsonProcessingException e) {
            log.error("json 처리 에러 : {}", e);
        }
    }

    @Async("taskExecutor")
    public void saveLocation(String payload) {
        // 데이터 처리 로직
        StringTokenizer stk = new StringTokenizer(payload);
        double longitude = Double.parseDouble(stk.nextToken());
        double latitude = Double.parseDouble(stk.nextToken());
        log.info("saveLocation 호출 : longitude {}, distanceList {}", longitude, latitude);

        // DB에서 현재 진행중인 rsvId 가져오기

        // DB rsv_route에 지금 위치 저장

        // rsvId로 DB rsv_detail에서 아직 탑승 안 한 지점 리스트 경유순서 순으로 받아오기

        // 현재 위치와 리스트의 각 지점 위치 계산

        // 거리 10m 이하면
        //  마지막이면 ROS 도착신호 전송, 3분 후 연결 해제 신호, 대기장소 데이터 전송
        //           FE 하차 버튼 활성화 신호, 도착 push알림 전송
        //  마지막 아니면 도착신호 전송
        //           FE 하차 버튼 활성화 신호, 도착 push알림 전송
        // 거리 500m 이하면 push 알림 전송
    }
}
