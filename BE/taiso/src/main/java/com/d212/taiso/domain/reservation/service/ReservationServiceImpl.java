package com.d212.taiso.domain.reservation.service;

import com.d212.taiso.domain.member.entity.Member;
import com.d212.taiso.domain.place.entity.Place;
import com.d212.taiso.domain.place.repository.PlaceRepository;
import com.d212.taiso.domain.reservation.dto.RsvAddReq;
import com.d212.taiso.domain.reservation.dto.RsvListRes;
import com.d212.taiso.domain.reservation.dto.RsvTogetherAddReq;
import com.d212.taiso.domain.reservation.entity.Reservation;
import com.d212.taiso.domain.reservation.mapper.ReservationMapper;
import com.d212.taiso.domain.reservation.repository.ReservationRepository;
import com.d212.taiso.global.result.error.ErrorCode;
import com.d212.taiso.global.result.error.exception.BusinessException;
import com.d212.taiso.global.util.CommonUtil;
import lombok.RequiredArgsConstructor;
import lombok.extern.log4j.Log4j2;
import org.springframework.stereotype.Service;

import java.time.LocalDateTime;
import java.util.List;

@Log4j2
@Service
@RequiredArgsConstructor
public class ReservationServiceImpl implements ReservationService {

    // Repository
    private final ReservationRepository reservationRepository;
    private final PlaceRepository placeRepository;

    // Mapper
    private final ReservationMapper reservationMapper;

    // 현재 회원 조회 용
    private final CommonUtil commonUtil;

    @Override
    public List<RsvListRes> getAllRsvList() {
        LocalDateTime currentTime = LocalDateTime.now();
        List<Reservation> reservationList = reservationRepository.findAllAfterCurrentTime(currentTime);

        return reservationList.stream()
                .map(reservationMapper::toRsvListRes)
                .toList();
    }

    @Override
    public List<RsvListRes> getMyRsvList() {
        return null;
    }

    @Override
    public void addRsv(RsvAddReq rsvAddReq) {

        // 요청한 멤버의 정보 가져오기
        Member member = commonUtil.getMember();

        Place place = null;

        // 즐겨찾기에 등록된 장소 ID를 통해 예약 시
        if (rsvAddReq.getPlaceId() != null) {
            // 해당 ID를 통해 장소 조회하기
            place = placeRepository.findPlaceById(rsvAddReq.getPlaceId())
                    .orElseThrow(() -> new BusinessException(ErrorCode.PLACE_ID_NOT_EXIST));

        } else {
            // 장소를 직접 입력 후에 예약 시
            place = Place.builder()
                    .latitude(rsvAddReq.getLatitude())
                    .longitude(rsvAddReq.getLongitude())
                    .address(rsvAddReq.getAddress())
                    .build();
        }

        Reservation reservation = Reservation.builder()
                .member(member)
                .place(place)
                .time(rsvAddReq.getTime())
                .cnt(rsvAddReq.getCnt())
                .build();

        // Todo 여기도 혹시 Place를 먼저? (조건이 뭐죠?) -> 내가 볼땐 기존 Id 없을 땐 저장 해줘야 됨.
        placeRepository.save(place);
        reservationRepository.save(reservation);

    }

    @Override
    public void addTogetherRsv(RsvTogetherAddReq rsvTogetherAddReq) {

    }
}
