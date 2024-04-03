package com.d212.taiso.domain.reservation.entity;

import static jakarta.persistence.FetchType.*;

import com.d212.taiso.domain.member.entity.Member;
import jakarta.persistence.Column;
import jakarta.persistence.EmbeddedId;
import jakarta.persistence.Entity;
import jakarta.persistence.JoinColumn;
import jakarta.persistence.ManyToOne;

import java.time.LocalDateTime;
import lombok.AccessLevel;
import lombok.AllArgsConstructor;
import lombok.Builder;
import lombok.Getter;
import lombok.NoArgsConstructor;

/**
 * Created by 전근렬 on 2024-03-21
 */
@Entity
@Getter
@NoArgsConstructor(access = AccessLevel.PROTECTED)
@AllArgsConstructor
@Builder
public class RsvDetail {

    // 복합키 (pk가 2개 이상이면 만듬)

    @EmbeddedId
    private RsvDetailId rsvDetailId;

    // 복합키 방식 2
//    @MapsId("rsvId")
//    @ManyToOne(fetch = LAZY)
//    @JoinColumn(name = "rsv_id")
//    private Reservation reservation;
//
//    @MapsId("placeId")
//    @ManyToOne(fetch = LAZY)
//    @JoinColumn(name = "place_id")
//    private Place place;

    @ManyToOne(fetch = LAZY)
    @JoinColumn(name = "email")
    private Member member;

    private int cnt; // 경유 인원

    @Column(name = "orders")
    private int orders; // 경유 순서

    @Builder.Default
    private boolean depart_flag = false; // 출발지 여부

    @Builder.Default
    private boolean board_flag = false; // 탑승 확인 여부

    @Builder.Default
    private boolean stop_flag = false; // 하차 확인 여부

    @Builder.Default
    private LocalDateTime arrivalTime = LocalDateTime.now();  // 도착 예정 시간 -> 예약 추가한 시간으로 용도 변경

    // orders 필드 값 설정하는 메서드
    public void setOrders(int orders) {
        this.orders = orders;
    }

    // board_flag 값 갱신하는 메서드
    public void setBoard_flag(boolean TF) {
        this.board_flag = TF;
    }
}
