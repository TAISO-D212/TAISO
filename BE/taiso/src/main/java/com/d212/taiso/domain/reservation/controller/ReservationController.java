package com.d212.taiso.domain.reservation.controller;

import com.d212.taiso.domain.route.service.RsvRouteService;
import com.d212.taiso.global.result.ResultResponse;
import lombok.RequiredArgsConstructor;
import lombok.extern.log4j.Log4j2;
import org.springframework.http.ResponseEntity;
import org.springframework.web.bind.annotation.GetMapping;
import org.springframework.web.bind.annotation.RequestMapping;
import org.springframework.web.bind.annotation.RestController;

/**
 * Created by 배성연 on 2024-03-22
 */

@Log4j2
@RestController
@RequestMapping("/rsv")
@RequiredArgsConstructor
public class ReservationController {

    private final RsvRouteService rsvRouteService;

    @GetMapping("/{connectiontest}")
    public ResponseEntity<ResultResponse> mqttConnectionTest() {
        try {
            rsvRouteService.locationToRoute();
            return ResponseEntity.ok().build();
        } catch (Exception e) {

            return null;
        }
    }

}
