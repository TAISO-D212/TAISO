//package com.d212.taiso.domain.route.service;
///**
// * Created by 배성연 on 2024-03-21
// */
//
//import com.d212.taiso.domain.route.dto.LocationDto;
//import com.d212.taiso.domain.route.entity.RsvRoute;
//import com.d212.taiso.domain.route.repository.RsvRouteRepository;
//import lombok.RequiredArgsConstructor;
//import lombok.extern.log4j.Log4j2;
//import org.springframework.stereotype.Service;
//import java.util.LinkedList;
//import java.util.List;
//
//@Log4j2
//@Service
//@RequiredArgsConstructor
//public class RsvRouteServiceImpl implements RsvRouteService {
//
//    private final RsvRouteRepository rsvRouteRepository;
//
//    private final AsyncService asyncService;
//
//    public RsvRoute updateRsvRoute(RsvRoute rsvRoute) {
//        return rsvRouteRepository.save(rsvRoute);
//    }
//
//    @Override
//    public void locationToRoute() {
//        // 임시 위치 데이터 생성 : 출발지, 경유지1, 목적지, 새로운 경유지
//        List<LocationDto> locations = new LinkedList<>();
//        locations.add(new LocationDto(37.23925005946422, 126.77316881517658, "마을회관"));
//        locations.add(new LocationDto(37.23925005935768, 126.77316881423511, "5일장"));
//        locations.add(new LocationDto(37.23925002345112, 126.77316876254113, "새마을금고"));
//        locations.add(new LocationDto(37.23925001223245, 126.77316853215425, "우리집"));
//
//        // 이후 AsyncService의 locationToRoute 호출
//        asyncService.locationToRoute(locations, 3);
//    }
//}
