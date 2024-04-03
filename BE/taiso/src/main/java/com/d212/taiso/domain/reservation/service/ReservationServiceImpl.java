package com.d212.taiso.domain.reservation.service;

import com.d212.taiso.domain.bookmark.entity.Bookmark;
import com.d212.taiso.domain.bookmark.repository.BookmarkRepository;
import com.d212.taiso.domain.member.entity.Member;
import com.d212.taiso.domain.place.entity.Place;
import com.d212.taiso.domain.place.repository.PlaceRepository;
import com.d212.taiso.domain.reservation.dto.MyRsvListRes;
import com.d212.taiso.domain.reservation.dto.OriginRouteInfoDto;
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
import jakarta.persistence.EntityNotFoundException;
import java.util.Optional;
import java.util.StringTokenizer;
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
    private final BookmarkRepository bookmarkRepository;

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
        // 이렇게 너무 길어지면 원래 builder를 엔티티쪽으로 빼는 것이 좋음. (차후 수정 예정)
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

    // Todo 이거 시간 오름차순으로 구현하려고 했는데 시간순으로 조회를 하려면
    // Detail에 있는 arrivalTime로 조회를 해야 되는데 null값이라 의미 없음.
    // 그래서 arrivalTime를 time을 통해서 넣어주던가 해야됨.
    @Override
    public List<MyRsvListRes> getMyRsvList() {
        Member member = commonUtil.getMember();
        List<RsvDetail> rsvDetailList = rsvDetailRepository.findRsvDetailByMember(member);
        return rsvDetailList.stream()
            .map(this::mapToMyRsvListRes)
            .collect(Collectors.toList());
    }

    private MyRsvListRes mapToMyRsvListRes(RsvDetail rsvDetail) {
        Reservation reservation = rsvDetail.getRsvDetailId().getReservation();
        Place startPlace = rsvDetail.getRsvDetailId().getPlace();
        Place endPlace = reservation.getPlace();

        return MyRsvListRes.builder()
            .rsvId(reservation.getId())
            .startPlaceId(startPlace.getId())
            .startLatitude(startPlace.getLatitude())
            .startLongitude(startPlace.getLongitude())
            .startAddress(startPlace.getAddress())
            .endPlaceId(endPlace.getId())
            .endLatitude(endPlace.getLatitude())
            .endLongitude(endPlace.getLongitude())
            .endAddress(endPlace.getAddress())
            .time(reservation.getTime())
            .arrivalTime(reservation.getArrivalTime())
            .cnt(reservation.getCnt())
            .build();
    }

    @Override
    @Transactional
    public long[] addRsv(RsvAddReq rsvAddReq) {

        // 현재 시간대에 이미 예약이 있으면 리턴
        Optional<Reservation> existingReservation = reservationRepository.findCurrentTime(
            rsvAddReq.getTime());

        if (existingReservation.isPresent()) {
            return new long[]{0, 0};
        }

        // 요청한 멤버의 정보 가져오기
        Member member = commonUtil.getMember();

        Place startPlace = null;
        Place endPlace = null;

        // 즐겨찾기에 등록된 장소 ID를 통해 예약 시 (출발지)
        if (rsvAddReq.getStartBookmarkId() != null) {
            // 해당 북마크 ID를 통해 장소 조회하기

            Bookmark bookmark = bookmarkRepository.findById(rsvAddReq.getStartBookmarkId())
                .orElseThrow(() -> new BusinessException(ErrorCode.BOOKMARK_NOT_EXIST));

            startPlace = bookmark.getPlace();

        } else {
            // 장소를 직접 입력 후에 예약 시 (출발지)
            startPlace = Place.builder()
                .latitude(rsvAddReq.getStartLatitude())
                .longitude(rsvAddReq.getStartLongitude())
                .address(rsvAddReq.getStartAddress())
                .build();
        }

        // 즐겨찾기 ID를 통해 예약 시 (도착지)
        if (rsvAddReq.getEndBookmarkId() != null) {

            Bookmark bookmark = bookmarkRepository.findById(rsvAddReq.getEndBookmarkId())
                .orElseThrow(() -> new BusinessException(ErrorCode.BOOKMARK_NOT_EXIST));

            endPlace = bookmark.getPlace();

        } else {
            // 장소를 직접 입력 후에 예약 시 (도착지)
            endPlace = Place.builder()
                .latitude(rsvAddReq.getEndLatitude())
                .longitude(rsvAddReq.getEndLongitude())
                .address(rsvAddReq.getEndAddress())
                .build();
        }

        // 예약 리스트를 먼저 만들고 그 부가적인 detail를 만들어 줘야 됨.
        Reservation reservation = Reservation.builder()
            .member(member)
            .place(endPlace)
            .time(rsvAddReq.getTime())
            .cnt(rsvAddReq.getCnt())
            .build();

        // 해당 Reservation에 대한 detail 만들기 (depart_flag도 설정해주기)

        // 복합키 설정
        RsvDetailId rsvDetailId = RsvDetailId.builder()
            .place(startPlace)
            .reservation(reservation)
            .build();

        RsvDetail rsvDetail = RsvDetail.builder()
            .rsvDetailId(rsvDetailId)
            .member(member)
            .cnt(rsvAddReq.getCnt())
//            .depart_flag(true) // 출발지 안써
            .build();

        placeRepository.save(startPlace);
        placeRepository.save(endPlace);
        Reservation savedReservation = reservationRepository.save(reservation);
        rsvDetailRepository.save(rsvDetail);

        return new long[]{savedReservation.getId(), startPlace.getId()};
    }

    @Override
    @Transactional
    public long addTogetherRsv(Long rsvId, RsvTogetherAddReq rsvTogetherAddReq) {

        // 요청한 멤버의 정보 가져오기
        Member member = commonUtil.getMember();

        // 예약 정보 가져오기
        Reservation reservation = reservationRepository.findById(rsvId)
            .orElseThrow(() -> new BusinessException(ErrorCode.RESERVATION_NOT_EXIST));

        // 예약 인원 수가 초과되었을 시 저장 실패
        // 기존 예약 인원 수 + 경유지 예약 인원 수
        int totalCnt = reservation.getCnt() + rsvTogetherAddReq.getCnt();

        if (totalCnt > 4) {
            return 0;
        }

        // 이미 유저가 해당 시간대에 예약을 했는 경우.
        // 시간으로 할까 아니면 예약 id로 할까??
        RsvDetail existingRsvDetail = rsvDetailRepository.findRsvDetailByMemberAndTime(member,
                reservation.getTime())
            .orElse(null); // 예약이 없을 경우 null을 반환

        if (existingRsvDetail != null) {
            return 0;
        }

        Place place = null;

        // 즐겨찾기에 등록된 장소 ID를 통해 예약 시 (경유지)
        if (rsvTogetherAddReq.getBookmarkId() != null) {

            Bookmark bookmark = bookmarkRepository.findById(rsvTogetherAddReq.getBookmarkId())
                .orElseThrow(() -> new BusinessException(ErrorCode.BOOKMARK_NOT_EXIST));

            place = bookmark.getPlace();

        } else {
            // 장소를 직접 입력 후에 예약 시 (경유지)
            place = Place.builder()
                .latitude(rsvTogetherAddReq.getLatitude())
                .longitude(rsvTogetherAddReq.getLongitude())
                .address(rsvTogetherAddReq.getAddress())
                .build();
        }

        // 복합키 설정
        RsvDetailId rsvDetailId = RsvDetailId.builder()
            .place(place)
            .reservation(reservation)
            .build();

        RsvDetail rsvDetail = RsvDetail.builder()
            .rsvDetailId(rsvDetailId)
            .member(member)
            .cnt(rsvTogetherAddReq.getCnt())
            .build();

        // 경유지 수 + 1
        int stopCnt = reservation.getStopCnt() + 1;

        // 이런 변화는 저장 필요 x
        reservation.changeStopCnt(stopCnt);
        reservation.changeCnt(totalCnt);

        placeRepository.save(place);
        rsvDetailRepository.save(rsvDetail);

        return place.getId();

    }

    @Override
    public void deleteRsv(Long rsvId, Long placeId) {

//        // 요청한 멤버의 정보 가져오기
//        Member member = commonUtil.getMember();

        Reservation reservation = reservationRepository.findById(rsvId)
            .orElseThrow(() -> new BusinessException(ErrorCode.RESERVATION_NOT_EXIST));

        Place place = placeRepository.findById(placeId)
            .orElseThrow(() -> new BusinessException(ErrorCode.PLACE_ID_NOT_EXIST));

        // 이 2개로 그냥 build해서 복합키 만들고 조회하기
        // 복합키
        RsvDetailId rsvDetailId = RsvDetailId.builder()
            .reservation(reservation)
            .place(place)
            .build();

        // 먼저 detail 조회하기 (placeId 정보도 같이 이용)
        RsvDetail rsvDetail = rsvDetailRepository.findById(rsvDetailId)
            .orElseThrow(() -> new BusinessException(ErrorCode.RSV_DETAIL_NOT_EXIST));

        // detail 정보를 통해서 예약 리스트 내용 수정하기

        // 총 예약 인원 수 변경
        int cnt = reservation.getCnt() - rsvDetail.getCnt();

        // 경유지 수 변경
        int stopCnt = reservation.getStopCnt() - 1;

        // 거리 배열 저장값에서 해당 경유지 제거
        int tempstopCnt = stopCnt + 2;
        String originDistStr = reservation.getRouteDist();
        // 받아온 경유지 간 거리를 테이블로 만들기
        //          주차장     목적지     경유지 1   경유지 2
        // 주차장       0       500        400       0번
        // 목적지      500       0         300       1번
        // 경유지 1    400      300         0        2번
        // 새 경유지    0번      1번        2번        0
        int lastNum = stopCnt - 1;
        int[][] distTable = new int[stopCnt][stopCnt];
        StringTokenizer stk = new StringTokenizer(originDistStr);
        for (int i = 0; i < lastNum; i++) {
            for (int j = 0; j < lastNum; j++) {
                distTable[i][j] = Integer.parseInt(stk.nextToken());
            }
        }
        // 지울 placeId가 해당 rsvId로 저장된 것 중 몇 번째 순서인지 찾아오는 로직
        // TODO : 해당 로직으로 행, 열 지워서 하기

        // 총 인원 수와 경유지 수가 모두 0이 되면 예약 리스트 자체를 제거
        // Cascade 적용이 되어있나?? -> 되어있음.
        if (cnt == 0 && stopCnt == 0) {
            reservationRepository.deleteById(rsvId);
        } else {
            rsvDetailRepository.delete(rsvDetail);
            // 예약 변경 값 적용하기
            reservation.changeCnt(cnt);
            reservation.changeStopCnt(stopCnt);
        }
    }

    @Override
    public Long getCurrentReservationId() {
        return reservationRepository.findReservationIdByHour();
    }
}
