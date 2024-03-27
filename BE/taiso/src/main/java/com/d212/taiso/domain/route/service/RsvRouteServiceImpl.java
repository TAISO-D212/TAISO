package com.d212.taiso.domain.route.service;
/**
 * Created by 배성연 on 2024-03-21
 */

import com.d212.taiso.domain.place.entity.Place;
import com.d212.taiso.domain.reservation.entity.RsvDetail;
import com.d212.taiso.domain.place.repository.PlaceRepository;
import com.d212.taiso.domain.reservation.repository.RsvDetailRepository;
import com.d212.taiso.domain.route.dto.LocationDto;
import lombok.RequiredArgsConstructor;
import lombok.extern.log4j.Log4j2;
import org.springframework.stereotype.Service;
import java.util.ArrayList;
import java.util.List;

@Log4j2
@Service
@RequiredArgsConstructor
public class RsvRouteServiceImpl implements RsvRouteService {

    private final AsyncService asyncService;
    private final PlaceRepository placeRepository;
    private final RsvDetailRepository rsvDetailRepository;

    @Override
    public void locationToRoute(long rsvId, long placeId) {
        List<LocationDto> locations = new ArrayList<>();

        // place_id==1인 곳 gps 정보 기본으로 설정
        Place defaultStartPlace = placeRepository.findById(1L)
            .orElseThrow(() -> new RuntimeException("기본 대기 위치가 존재하지 않습니다."));
        locations.add(new LocationDto(defaultStartPlace.getId(), defaultStartPlace.getLatitude(),
            defaultStartPlace.getLongitude()));

        if (rsvId > 0) {
            // 기존 예약이면 기존 위치 데이터 받아와 locations.add에 넣기
            List<RsvDetail> rsvDetails = rsvDetailRepository.findByRsvId(rsvId);
            for (RsvDetail detail : rsvDetails) {
                Place place = placeRepository.findById(detail.getRsvDetailId().getPlace().getId())
                    .orElseThrow(() -> new RuntimeException("경유지 장소가 존재하지 않습니다."));
                locations.add(
                    new LocationDto(place.getId(), place.getLatitude(), place.getLongitude()));
            }
        }

        // 목적지 정보 추가
        Place destinationPlace = placeRepository.findById(placeId)
            .orElseThrow(() -> new RuntimeException("목적지 장소가 존재하지 않습니다."));
        locations.add(new LocationDto(destinationPlace.getId(), destinationPlace.getLatitude(),
            destinationPlace.getLongitude()));

        // 이후 AsyncService의 locationToRoute 호출
        log.info("locations 완성, rsvId : {}, placeId : {}, locations : {}", rsvId, placeId,
            locations);
        asyncService.locationToRoute(locations, rsvId, placeId);
    }


}
