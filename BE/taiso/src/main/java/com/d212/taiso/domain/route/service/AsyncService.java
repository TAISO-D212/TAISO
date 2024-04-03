package com.d212.taiso.domain.route.service;

import com.d212.taiso.domain.member.service.AlertService;
import com.d212.taiso.domain.place.entity.Place;
import com.d212.taiso.domain.place.repository.PlaceRepository;
import com.d212.taiso.domain.reservation.dto.OriginRouteInfoDto;
import com.d212.taiso.domain.reservation.entity.Reservation;
import com.d212.taiso.domain.reservation.entity.RsvDetail;
import com.d212.taiso.domain.reservation.repository.ReservationRepository;
import com.d212.taiso.domain.reservation.repository.RsvDetailRepository;
import com.d212.taiso.domain.reservation.service.ReservationService;
import com.d212.taiso.domain.route.dto.LocationDto;
import com.d212.taiso.domain.route.dto.LocationPayload;
import com.d212.taiso.domain.route.entity.RsvRoute;
import com.d212.taiso.domain.route.mqtt.Publisher;
import com.d212.taiso.domain.route.repository.RsvRouteRepository;
import com.fasterxml.jackson.core.JsonProcessingException;
import com.fasterxml.jackson.databind.ObjectMapper;
import com.google.firebase.messaging.FirebaseMessagingException;
import jakarta.persistence.EntityNotFoundException;
import jakarta.transaction.Transactional;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.StringTokenizer;
import java.util.concurrent.CompletableFuture;
import lombok.RequiredArgsConstructor;
import lombok.extern.log4j.Log4j2;
import org.springframework.scheduling.annotation.Async;
import org.springframework.stereotype.Service;

/**
 * Created by 배성연 on 2024-03-21
 */

@Log4j2
@Service
@RequiredArgsConstructor
public class AsyncService {

    private final Publisher publisher;
    private final ObjectMapper objectMapper;
    private final AlertService alertService;
    private final ReservationService reservationService;
    private final PlaceRepository placeRepository;
    private final RsvRouteRepository rsvRouteRepository;
    private final RsvDetailRepository rsvDetailRepository;
    private final ReservationRepository reservationRepository;

    @Transactional
    @Async("taskExecutor")
    public CompletableFuture<Void> locationToRoute(List<LocationDto> locations, long rsvId,
        long placeId) {
        // location 데이터 처리 후 MQTT 메시지로 전송
        try {
            // 데이터 처리
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
        // 받아온 거리 데이터 처리 후 최소 경로 계산, 30km 안 넘으면 순서 저장 / 넘으면 삭제
        try {
            LocationPayload locationPayload = objectMapper.readValue(payload,
                LocationPayload.class);
            long rsvId = locationPayload.getRsvId();
            long placeId = locationPayload.getPlaceId();
            int re = locationPayload.getDistanceList().size();
            int[] distanceList = new int[re];

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
            if (originDistStr == null) {
                // 신규 예약 시 -> [새 경유지-주차장, 새 경유지-목적지, 주차장-목적지 거리]
                //          주차장     목적지     새 경유지
                // 주차장               2번          0번
                // 목적지      2번                   1번
                // 새 경유지   0번       1번
                distTable = new int[stopCnt][stopCnt];
                distTable[0][2] = distanceList[0];
                distTable[2][0] = distanceList[0];
                distTable[1][2] = distanceList[1];
                distTable[2][1] = distanceList[1];
                distTable[0][1] = distanceList[2];
                distTable[1][0] = distanceList[2];

            } else {
                // 경유지 추가 시 -> [새 경유지와 주차장, 목적지, 경유지1까지 각각 거리]
                // 받아온 경유지 간 거리를 테이블로 만들기
                //          주차장     목적지     경유지 1   새 경유지
                // 주차장       0       500        400       0번
                // 목적지      500       0         300       1번
                // 경유지 1    400      300         0        2번
                // 새 경유지    0번      1번        2번        0
                int lastNum = stopCnt - 1;
                distTable = new int[stopCnt][stopCnt];
                StringTokenizer stk = new StringTokenizer(originDistStr);
                for (int i = 0; i < lastNum; i++) {
                    for (int j = 0; j < lastNum; j++) {
                        distTable[i][j] = Integer.parseInt(stk.nextToken());
                    }
                }
                for (int i = 0; i < lastNum; i++) {
                    distTable[lastNum][i] = distanceList[i];
                    distTable[i][lastNum] = distanceList[i];
                }
            }

            // distance로 경로 계산 : 외판원 순회 알고리즘 활용
            PathResult result = findPath(distTable, 0, 1);
            log.info("경로 계산 완료 : {}", Arrays.toString(result.path.toArray()));
            log.info("총 거리 : {}", result.totalDistance);

            if (result.totalDistance > 30000) {
                // 최소 거리 총합이 30km 넘으면 예약 삭제 DB 업데이트, 예약 실패 push 알림
                log.info("30km 초과! 예약 실패");
                reservationService.deleteRsv(rsvId, placeId);
                // TODO : 예약 실패 push 알림 호출 추가
            } else {
                // 최소 거리 총합이 30km 이내면 예약 성공 DB 업데이트, 예약 성공 push 알림
                log.info("30km 이하! 예약 성공");
                // 예약 성공 DB 업데이트 - route_dist update
                StringBuilder sb = new StringBuilder();
                for (int i = 0; i < stopCnt; i++) {
                    for (int j = 0; j < stopCnt; j++) {
                        sb.append(distTable[i][j] + " ");
                    }
                }
                int temp = reservationRepository.updateRouteDistByRsvId(sb.toString(), rsvId);
                if (temp > 0) {
                    log.info("route dist updated.");
                } else {
                    log.error("route dist update 실패!");
                }
                // 경로 순서대로 orders 업데이트
                List<RsvDetail> rsvDetailList = rsvDetailRepository.findRdByRsvIdOrderByArrivalTime(
                    rsvId);
                for (int i = 1; i < stopCnt - 1; i++) {
                    int order = result.path.get(i) - 1;
                    rsvDetailList.get(i - 1).setOrders(order);
                    rsvDetailRepository.save(rsvDetailList.get(i - 1));
                    log.info((i - 1) + " 번째 저장한 rsvDetail order update to " + order);
                }
                log.info("rsvDetail order updated!");
                // TODO : 예약 성공 push 알림 호출 추가
            }

        } catch (JsonProcessingException e) {
            log.error("json 처리 에러 : {}", e);
        }
    }

    @Transactional
    @Async("taskExecutor")
    public void saveLocation(String payload) {
        // 데이터 처리 로직
        StringTokenizer stk = new StringTokenizer(payload);
        double longitude = Double.parseDouble(stk.nextToken());
        double latitude = Double.parseDouble(stk.nextToken());
        log.info("saveLocation 호출 : longitude {}, latitude {}", longitude, latitude);

        // DB에서 현재 진행중인 rsvId 가져오기
        Long rsvId = reservationRepository.findReservationIdByHour();
        log.info("현재 진행중인 rsvId : {}", rsvId);

        // DB rsv_route에 지금 위치 저장
        Optional<Reservation> reservationOptional = reservationRepository.findById(rsvId);
        if (reservationOptional.isPresent()) {
            log.info(reservationOptional.get());
            RsvRoute rsvRoute = RsvRoute.builder()
                .reservation(reservationOptional.get())
                .latitude(latitude)
                .longitude(longitude)
                .build();
            rsvRouteRepository.save(rsvRoute);
            log.info("현재 위치 저장 성공!");
        } else {
            log.error("현재 위치 저장 실패!");
        }
        // rsvId로 DB rsv_detail에서 경유순서 순으로 아직 탑승 안 한 다음 지점이나 목적지 받아오기
        double nextLatitude = 0;
        double nextLongitude = 0;
        RsvDetail next = rsvDetailRepository.findNextRsvDetail(rsvId);
        if (next != null) {
            nextLatitude = next.getRsvDetailId().getPlace().getLatitude();
            nextLongitude = next.getRsvDetailId().getPlace().getLongitude();
        } else {
            Long placeId = reservationRepository.findPlaceIdByReservationId(rsvId);
            Optional<Place> nextPlace = placeRepository.findPlaceById(placeId);
            nextLatitude = nextPlace.get().getLatitude();
            nextLongitude = nextPlace.get().getLongitude();
        }
        // 현재 위치와 리스트의 각 지점 위치 계산
        int distance = calculateDistanceInMeters(latitude, longitude, nextLatitude, nextLongitude);

        // 거리 200m 이하면
        if (distance < 200 && distance > 150) {
            log.info("거리 200m 이하, 도착 예정 push 알림 전송");
            try {
                alertService.soonAlertSend(rsvId);
            } catch (InterruptedException | FirebaseMessagingException e) {
                log.error("push 알림 에러 : {}", e);
            }
        }

        // 거리 10m 이하면 push 알림 전송
        if (distance < 50 && distance > 0) {
            log.info("거리 50m 이하, 도착 push 알림 전송");
            try {
                alertService.arrivalAlertSend(rsvId);
            } catch (InterruptedException e) {
                log.error("push 알림 에러 : {}", e);
            } catch (FirebaseMessagingException e) {
                throw new RuntimeException(e);
            }
            if (next == null) {
                // 마지막이면 ROS 연결 해제 신호
                // 도착 push알림 전송
                log.info("endConnection");
                publisher.publishLocations("connect/BE/end", "connection Ended.");
            } else {
                next.setBoard_flag(true);
                rsvDetailRepository.save(next);
            }
        }
    }

    private static int calculateDistance(List<Integer> path, int[][] distTable) {
        int totalDistance = 0;
        for (int i = 0; i < path.size() - 1; i++) {
            totalDistance += distTable[path.get(i)][path.get(i + 1)];
        }
        return totalDistance;
    }

    private static PathResult findPath(int[][] distTable, int start, int end) {
        int N = distTable.length;
        List<Integer> nodes = new ArrayList<>();
        for (int i = 0; i < N; i++) {
            if (i != start && i != end) {
                nodes.add(i);
            }
        }

        int minDistance = Integer.MAX_VALUE;
        List<Integer> minPath = new ArrayList<>();

        // 방문할 모든 순열 구한 후 탐색
        do {
            List<Integer> path = new ArrayList<>();
            path.add(start);
            path.addAll(nodes);
            path.add(end);
            int distance = calculateDistance(path, distTable);

            if (distance < minDistance) {
                minDistance = distance;
                minPath = new ArrayList<>(path);
            }
        } while (nextPermutation(nodes));

        return new PathResult(minPath, minDistance);
    }

    // 리스트의 순열 중 사전적으로 다음에 오는 순열을 구하는 메서드
    public static <T extends Comparable<? super T>> boolean nextPermutation(List<T> list) {
        // 리스트의 끝에서부터 시작하여 현재 요소가 이전 요소보다 크기 시작하는 첫 번째 지점 찾기
        int i = list.size() - 2;
        while (i >= 0 && list.get(i).compareTo(list.get(i + 1)) >= 0) {
            i--;
        }

        if (i == -1) {
            return false; // 이미 최대 순열에 도달했음
        }

        // 리스트의 끝에서부터 시작하여 앞에서 찾은 요소보다 큰 첫 번째 요소를 찾습니다.
        int j = list.size() - 1;
        while (list.get(i).compareTo(list.get(j)) >= 0) {
            j--;
        }

        // 이 두 요소의 위치를 바꿉니다.
        Collections.swap(list, i, j);

        // 앞에서 찾은 요소의 오른쪽에 있는 모든 요소들을 뒤집습니다.
        Collections.reverse(list.subList(i + 1, list.size()));
        return true;
    }

    public static int calculateDistanceInMeters(double lat1, double lon1, double lat2,
        double lon2) {
        int EARTH_RADIUS = 6371000; // 지구 반지름

        // 위도, 경도를 라디안으로 변환
        double dLat = Math.toRadians(lat2 - lat1);
        double dLon = Math.toRadians(lon2 - lon1);

        // 변환된 라디안 값을 사용
        lat1 = Math.toRadians(lat1);
        lat2 = Math.toRadians(lat2);

        // Haversine 공식
        double a = Math.pow(Math.sin(dLat / 2), 2)
            + Math.pow(Math.sin(dLon / 2), 2)
            * Math.cos(lat1) * Math.cos(lat2);
        double c = 2 * Math.atan2(Math.sqrt(a), Math.sqrt(1 - a));

        int distance = (int) (EARTH_RADIUS * c);

        // 미터 단위로 결과 변환
        return distance;
    }

    static class PathResult {  // 경로 알고리즘 결과 저장용 클래스

        List<Integer> path;
        int totalDistance;

        public PathResult(List<Integer> path, int totalDistance) {
            this.path = path;
            this.totalDistance = totalDistance;
        }

        @Override
        public String toString() {
            return "path : " + Arrays.toString(path.toArray()) + ", totalDistance : "
                + totalDistance;
        }
    }

}



