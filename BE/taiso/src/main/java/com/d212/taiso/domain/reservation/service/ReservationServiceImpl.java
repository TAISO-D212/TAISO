package com.d212.taiso.domain.reservation.service;

import com.d212.taiso.domain.member.entity.Member;
import com.d212.taiso.domain.place.entity.Place;
import com.d212.taiso.domain.place.repository.PlaceRepository;
import com.d212.taiso.domain.reservation.dto.RsvAddReq;
import com.d212.taiso.domain.reservation.dto.RsvListRes;
import com.d212.taiso.domain.reservation.dto.RsvTogetherAddReq;
import com.d212.taiso.domain.reservation.entity.Reservation;
import com.d212.taiso.domain.reservation.entity.RsvDetail;
import com.d212.taiso.domain.reservation.entity.RsvDetailId;
import com.d212.taiso.domain.reservation.mapper.ReservationMapper;
import com.d212.taiso.domain.reservation.repository.ReservationRepository;
import com.d212.taiso.domain.reservation.repository.RsvDetailRepository;
import com.d212.taiso.global.result.error.ErrorCode;
import com.d212.taiso.global.result.error.exception.BusinessException;
import com.d212.taiso.global.util.CommonUtil;
import java.util.stream.Collectors;
import lombok.RequiredArgsConstructor;
import lombok.extern.log4j.Log4j2;
import org.springframework.stereotype.Service;

import java.time.LocalDateTime;
import java.util.List;
import org.springframework.transaction.annotation.Transactional;

@Log4j2
@Service
@RequiredArgsConstructor
public class ReservationServiceImpl implements ReservationService {

    // Repository
    private final ReservationRepository reservationRepository;
    private final PlaceRepository placeRepository;
    private final RsvDetailRepository rsvDetailRepository;

    // Mapper
    private final ReservationMapper reservationMapper;

    // 현재 회원 조회 용
    private final CommonUtil commonUtil;

    @Override
    public List<RsvListRes> getAllRsvList() {
        LocalDateTime currentTime = LocalDateTime.now();
        List<Reservation> reservationList = reservationRepository.findAllAfterCurrentTime(
            currentTime);

        // 그냥 객체를 넣으면 망하는구나...
        return reservationList
            .stream()
            .map(reservation -> RsvListRes
                .builder()
                .rsvId(reservation.getId())
                .placeId(reservation.getPlace().getId())
                .latitude(reservation.getPlace().getLatitude())
                .longitude(reservation.getPlace().getLongitude())
                .address(reservation.getPlace().getAddress())
                .time(reservation.getTime())
                .arrivalTime(reservation.getArrivalTime())
                .stopCnt(reservation.getStopCnt())
                .cnt(reservation.getCnt())
                .build())
            .collect(Collectors.toList());

        // Todo 시간 나면 이것도 구현해보기(연습)
        // Mapper (망한 버전, 객체로 안 했더라면 가능은 하게 만들 수 있었음.)
//        return reservationList.stream()
//            .map(reservationMapper::toRsvListRes)
//            .toList();
    }

    @Override
    public List<RsvListRes> getMyRsvList() {
        return null;
    }

    @Override
    @Transactional
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
    @Transactional
    public void addTogetherRsv(Long rsvId, RsvTogetherAddReq rsvTogetherAddReq) {

        // 요청한 멤버의 정보 가져오기
        Member member = commonUtil.getMember();

        // 예약 정보 가져오기
        Reservation reservation = reservationRepository.findById(rsvId)
            .orElseThrow(() -> new BusinessException(ErrorCode.RESERVATION_NOT_EXIST));

        Place place = null;

        // 즐겨찾기에 등록된 장소 ID를 통해 예약 시 (경유지)
        if (rsvTogetherAddReq.getPlaceId() != null) {
            place = placeRepository.findPlaceById(rsvTogetherAddReq.getPlaceId())
                .orElseThrow(() -> new BusinessException(ErrorCode.PLACE_ID_NOT_EXIST));
        } else {
            // 장소를 직접 입력 후에 예약 시 (경유지)
            place = Place.builder()
                .latitude(rsvTogetherAddReq.getLatitude())
                .longitude(rsvTogetherAddReq.getLongitude())
                .address(rsvTogetherAddReq.getAddress())
                .build();
        }

        RsvDetailId rsvDetailId = RsvDetailId.builder()
            .place(place)
            .reservation(reservation)
            .build();

        RsvDetail rsvDetail = RsvDetail.builder()
            .rsvDetailId(rsvDetailId)
            .member(member)
            .build();

        // 경유지 수 + 1
        int stopCnt = reservation.getStopCnt() + 1;

        // 기존 예약 인원 수 + 경유지 예약 인원 수
        int cnt = reservation.getCnt() + rsvTogetherAddReq.getCnt();

        // 이렇게 변화 줬으면 저장 해줘야 되나??
        reservation.changeStopCnt(stopCnt);
        reservation.changeCnt(cnt);

        placeRepository.save(place);
        rsvDetailRepository.save(rsvDetail);

    }
}
