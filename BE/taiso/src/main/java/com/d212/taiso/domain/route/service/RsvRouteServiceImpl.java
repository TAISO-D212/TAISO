package com.d212.taiso.domain.route.service;
/**
 * Created by 배성연 on 2024-03-21
 */

import com.d212.taiso.domain.place.entity.Place;
import com.d212.taiso.domain.reservation.entity.RsvDetail;
import com.d212.taiso.domain.place.repository.PlaceRepository;
import com.d212.taiso.domain.reservation.repository.ReservationRepository;
import com.d212.taiso.domain.reservation.repository.RsvDetailRepository;
import com.d212.taiso.domain.route.dto.LocationDto;
import com.d212.taiso.domain.route.mqtt.Publisher;
import com.fasterxml.jackson.core.JsonProcessingException;
import com.fasterxml.jackson.databind.ObjectMapper;
import java.util.HashMap;
import java.util.Map;
import lombok.RequiredArgsConstructor;
import lombok.extern.log4j.Log4j2;
import org.springframework.stereotype.Service;
import java.util.ArrayList;
import java.util.List;

@Log4j2
@Service
@RequiredArgsConstructor
public class RsvRouteServiceImpl implements RsvRouteService {

    private final ObjectMapper objectMapper;
    private final Publisher publisher;
    private final AsyncService asyncService;
    private final PlaceRepository placeRepository;
    private final RsvDetailRepository rsvDetailRepository;
    private final ReservationRepository reservationRepository;

    @Override
    public void startConnection(long rsvId) {
        // 경유지 gps 리스트 ROS 넘겨주기 -> ROS단에서는 ros토픽으로 저장
        log.info("startConnection - rsvId : {}", rsvId);
        try {
            List<LocationDto> locations = new ArrayList<>();

            // place_id==1인 곳 gps 정보 기본으로 설정
            log.info("defaultStartPlace 로드");
            Place defaultStartPlace = placeRepository.findById(1L)
                .orElseThrow(() -> new RuntimeException("기본 대기 위치가 존재하지 않습니다."));
            locations.add(
                new LocationDto(defaultStartPlace.getId(), defaultStartPlace.getLatitude(),
                    defaultStartPlace.getLongitude()));

            // 경유 순서대로 가져오기
            log.info("rsvDetails 로드");
            List<RsvDetail> rsvDetails = rsvDetailRepository.findRdByRsvId(rsvId);
            for (RsvDetail detail : rsvDetails) {
                Place place = placeRepository.findById(detail.getRsvDetailId().getPlace().getId())
                    .orElseThrow(() -> new RuntimeException("경유지 장소가 존재하지 않습니다."));
                locations.add(
                    new LocationDto(place.getId(), place.getLatitude(), place.getLongitude()));
            }

            // 목적지 gps 정보 가져오기
            log.info("destPlace 로드");
            Long destPlaceId = reservationRepository.findPlaceIdByReservationId(rsvId);
            Place destPlace = placeRepository.findById(destPlaceId)
                .orElseThrow(() -> new RuntimeException("목적지 위치가 존재하지 않습니다."));
            locations.add(
                new LocationDto(destPlace.getId(), destPlace.getLatitude(),
                    destPlace.getLongitude()));

            // 데이터 처리 후 MQTT 연결 메시지 발행
            log.info("MQTT 메시지 발행");
            Map<String, Object> dataMap = new HashMap<>();
            dataMap.put("rsvId", rsvId);
            dataMap.put("locations", locations);
            String payload = objectMapper.writeValueAsString(dataMap);
            publisher.publishLocations("connect/BE/start", payload);
            log.info("publish 완료 - locations : {}", locations);

        } catch (JsonProcessingException e) {
            log.error("connection 중 데이터 처리 과정 Json 에러 : {}", e);
        }
    }

    @Override
    public void endConnection() {
        log.info("endConnection");
        publisher.publishLocations("connect/BE/end", "connection Ended.");
    }

    @Override
    public void locationToRoute(long rsvId, long placeId) {
        // DB에서 location 정보 꺼내와 ROS에 publish
        List<LocationDto> locations = new ArrayList<>();

        // place_id==1인 곳 gps 정보 기본으로 설정
        Place defaultStartPlace = placeRepository.findById(1L)
            .orElseThrow(() -> new RuntimeException("기본 대기 위치가 존재하지 않습니다."));
        locations.add(new LocationDto(defaultStartPlace.getId(), defaultStartPlace.getLatitude(),
            defaultStartPlace.getLongitude()));

        // 목적지 gps 정보 가져오기
        Long destPlaceId = reservationRepository.findPlaceIdByReservationId(rsvId);
        Place destPlace = placeRepository.findById(destPlaceId)
            .orElseThrow(() -> new RuntimeException("목적지 위치가 존재하지 않습니다."));
        locations.add(
            new LocationDto(destPlace.getId(), destPlace.getLatitude(), destPlace.getLongitude()));

        // 새로운 경유지도 우선 rsvDetail에 추가되므로 추가 순서대로 가져오기
        List<RsvDetail> rsvDetails = rsvDetailRepository.findRdByRsvIdOrderByArrivalTime(rsvId);
        for (RsvDetail detail : rsvDetails) {
            Place place = placeRepository.findById(detail.getRsvDetailId().getPlace().getId())
                .orElseThrow(() -> new RuntimeException("경유지 장소가 존재하지 않습니다."));
            locations.add(
                new LocationDto(place.getId(), place.getLatitude(), place.getLongitude()));
        }

        // 이후 AsyncService의 locationToRoute 호출
        log.info("locations 완성, rsvId : {}, placeId : {}, locations : {}", rsvId, placeId,
            locations);
        asyncService.locationToRoute(locations, rsvId, placeId);
    }


}
