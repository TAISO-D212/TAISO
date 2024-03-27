package com.d212.taiso.domain.route.service;
/**
 * Created by 배성연 on 2024-03-21
 */

import com.d212.taiso.domain.route.dto.LocationDto;
import com.d212.taiso.domain.route.entity.RsvRoute;
import com.d212.taiso.domain.route.repository.RsvRouteRepository;
import lombok.RequiredArgsConstructor;
import lombok.extern.log4j.Log4j2;
import org.springframework.stereotype.Service;
import java.util.LinkedList;
import java.util.List;

@Log4j2
@Service
@RequiredArgsConstructor
public class RsvRouteServiceImpl implements RsvRouteService {

    private final AsyncService asyncService;

    @Override
    public void locationToRoute(long rsvId, long placeId) {
        List<LocationDto> locations = new LinkedList<>();

        if (rsvId > 0) {
            // 신규예약 아니면 기존 위치 데이터 받아와 locations.add에 넣기

        } else {
            // 신규예약이면 이전 시간 예약 있는지 확인, 있으면 이전 목적지 없으면 마을회관 주소 받아와 locations에 add

            // 임시 위치 데이터 생성 : 출발지, 경유지1, 새로운 경유지, 목적지
            locations.add(new LocationDto(1, 37.23925005946422, 126.77316881517658));
            locations.add(new LocationDto(2, 37.23925005935768, 126.77316881423511));
            locations.add(new LocationDto(3, 37.23925002345112, 126.77316876254113));
            locations.add(new LocationDto(4, 37.23925001223245, 126.77316853215425));
        }

        // 이후 AsyncService의 locationToRoute 호출
        asyncService.locationToRoute(locations, rsvId, placeId);
    }


}
