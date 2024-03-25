package com.d212.taiso.domain.reservation.controller;

import com.d212.taiso.domain.reservation.dto.RsvAddReq;
import com.d212.taiso.domain.reservation.dto.RsvListRes;
import com.d212.taiso.domain.reservation.service.ReservationService;
import com.d212.taiso.domain.route.service.RsvRouteService;
import com.d212.taiso.global.result.ResultCode;
import com.d212.taiso.global.result.ResultResponse;
import lombok.RequiredArgsConstructor;
import lombok.extern.log4j.Log4j2;
import org.springframework.http.ResponseEntity;
import org.springframework.web.bind.annotation.*;

import java.util.List;

/**
 * Created by 배성연 on 2024-03-22
 */

@Log4j2
@RestController
@RequestMapping("/reservations")
@RequiredArgsConstructor
public class ReservationController {

    private final RsvRouteService rsvRouteService;

    private final ReservationService reservationService;

    @GetMapping("/{connectiontest}")
    public ResponseEntity<ResultResponse> mqttConnectionTest() {
        try {
            // 임시 새 경유지
            rsvRouteService.locationToRoute(0, 2);
            return ResponseEntity.ok().build();
        } catch (Exception e) {

            return null;
        }
    }

    @GetMapping("/")
    public ResponseEntity<ResultResponse> getAllRsvList() {
        List<RsvListRes> rsvListResList = reservationService.getAllRsvList();
        return ResponseEntity.ok(
            ResultResponse.of(ResultCode.GET_RESERVATION_SUCCESS, rsvListResList));
    }

    @PostMapping("/")
    public ResponseEntity<ResultResponse> addRsv(@RequestBody RsvAddReq rsvAddReq) {
        reservationService.addRsv(rsvAddReq);
        return ResponseEntity.ok(ResultResponse.of(ResultCode.ADD_RESERVATION_SUCCESS, true));
    }
}
