package com.d212.taiso.domain.reservation.entity;

import static jakarta.persistence.FetchType.*;

import com.d212.taiso.domain.member.entity.Member;
import com.d212.taiso.domain.place.entity.Place;
import jakarta.persistence.Column;
import jakarta.persistence.Entity;
import jakarta.persistence.GeneratedValue;
import jakarta.persistence.GenerationType;
import jakarta.persistence.Id;
import jakarta.persistence.JoinColumn;
import jakarta.persistence.ManyToOne;
import java.time.LocalDateTime;
import lombok.AccessLevel;
import lombok.AllArgsConstructor;
import lombok.Builder;
import lombok.Getter;
import lombok.NoArgsConstructor;
import org.springframework.data.annotation.CreatedDate;

/**
 * Created by 전근렬 on 2024-03-21
 */

@Entity
@Getter
@AllArgsConstructor
@NoArgsConstructor(access = AccessLevel.PROTECTED)
@Builder
public class Reservation {

    @Id
    @GeneratedValue(strategy = GenerationType.IDENTITY)
    @Column(name = "rsv_id")
    private Long id; // 예약 번호

    @ManyToOne(fetch = LAZY)
    @JoinColumn(name = "email") // fk키 = email
    private Member member;

    @ManyToOne(fetch = LAZY)
    @JoinColumn(name = "place_id") // 도착지 아이디
    private Place place;

    private LocalDateTime time; // 예약 일시 (출발 시간)

    @Builder.Default
    private int stopCnt = 1; // 경유지 수

    @Builder.Default
    private int cnt = 1; // 예약 인원 수 (총 인원)

    private LocalDateTime arrivalTime; // 예상 종료 시간

    private String routeImg; // 총 경로 이미지

    private String routeDist; // 경유지 간 거리

    public void changeStopCnt(int stopCnt) {
        this.stopCnt = stopCnt;
    }

    public void changeCnt(int cnt) {
        this.cnt = cnt;
    }


}
