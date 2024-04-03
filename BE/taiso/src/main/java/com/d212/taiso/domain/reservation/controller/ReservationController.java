package com.d212.taiso.domain.reservation.controller;

import com.d212.taiso.domain.member.service.AlertService;
import com.d212.taiso.domain.member.service.MemberService;
import com.d212.taiso.domain.reservation.dto.MyRsvListRes;
import com.d212.taiso.domain.reservation.dto.RsvAddReq;
import com.d212.taiso.domain.reservation.dto.RsvListRes;
import com.d212.taiso.domain.reservation.dto.RsvTogetherAddReq;
import com.d212.taiso.domain.reservation.service.ReservationService;
import com.d212.taiso.domain.route.service.RsvRouteService;
import com.d212.taiso.global.result.ResultCode;
import com.d212.taiso.global.result.ResultResponse;
import lombok.RequiredArgsConstructor;
import lombok.extern.log4j.Log4j2;
import org.springframework.http.ResponseEntity;
import org.springframework.scheduling.annotation.Scheduled;
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
    private final MemberService memberService;
    private final AlertService alertService;

    // 매 시간마다 자동으로 예약 확인 후 있다면 실행
    @Scheduled(cron = "0 0 0/1 * * *", zone = "Asia/Seoul")
    @GetMapping("/connection")
    public ResponseEntity<ResultResponse> mqttConnectionTest() {
        try {
            rsvRouteService.endConnection();
            Long rsvId = reservationService.getCurrentReservationId();
            if (rsvId != null) {
                log.info("예약 있음. 연결을 시작합니다.");
                rsvRouteService.startConnection(rsvId);
                alertService.departAlertSend(rsvId);
                return ResponseEntity.ok().build();
            } else {
                log.info("예약 없음. 연결하지 않습니다.");
                return ResponseEntity.ok().build();
            }
        } catch (Exception e) {
            return null;
        }
    }

    // 연결 강제 중단
    @GetMapping("/connectionEnd")
    public ResponseEntity<ResultResponse> mqttEndTest() {
        try {
            rsvRouteService.endConnection();
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

    // 내 예약 목록 가져오기
    @GetMapping("/my")
    public ResponseEntity<ResultResponse> getAllMyRsvList() {
        List<MyRsvListRes> myRsvListResList = reservationService.getMyRsvList();
        return ResponseEntity.ok(
            ResultResponse.of(ResultCode.GET_MY_RESERVATION_SUCCESS, myRsvListResList));
    }

    @PostMapping("/")
    public ResponseEntity<ResultResponse> addRsv(@RequestBody RsvAddReq rsvAddReq) {
        long[] addResult = reservationService.addRsv(rsvAddReq);
        long rsvId = addResult[0];
        long placeId = addResult[1];
        if (placeId == 0) {
            return ResponseEntity.ok(
                ResultResponse.of(ResultCode.ADD_RESERVATION_FAIL, "이미 해당 시간에 예약이 있습니다."));
        } else {
            rsvRouteService.locationToRoute(rsvId, placeId);
            return ResponseEntity.ok(
                ResultResponse.of(ResultCode.ADD_RESERVATION_SUCCESS, "예약이 성공적으로 신청되었습니다."));
        }

    }

    @PostMapping("/{rsvId}")
    public ResponseEntity<ResultResponse> addTogetherRsv(@PathVariable("rsvId") Long rsvId,
        @RequestBody
        RsvTogetherAddReq rsvTogetherAddReq) {
        long placeId = reservationService.addTogetherRsv(rsvId, rsvTogetherAddReq);

        if (placeId == 0) {
            return ResponseEntity.ok(
                ResultResponse.of(ResultCode.ADD_RESERVATION_FAIL, "합승 예약에 실패했습니다.")
            );
        } else {
            rsvRouteService.locationToRoute(rsvId, placeId);
            return ResponseEntity.ok(
                ResultResponse.of(ResultCode.ADD_TOGETHER_RESERVATION_SUCCESS,
                    "합승 예약이 성공적으로 신청되었습니다."));
        }
    }

    @DeleteMapping("/{rsvId}/{placeId}")
    public ResponseEntity<ResultResponse> deleteRsv(@PathVariable("rsvId") Long rsvId,
        @PathVariable("placeId") Long placeId) {
        reservationService.deleteRsv(rsvId, placeId);
        return ResponseEntity.ok(ResultResponse.of(ResultCode.DELETE_RESERVATION_SUCCESS, true));
    }
}
