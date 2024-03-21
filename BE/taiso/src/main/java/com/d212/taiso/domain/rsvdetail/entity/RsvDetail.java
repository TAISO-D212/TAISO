package com.d212.taiso.domain.rsvdetail.entity;

import static jakarta.persistence.FetchType.*;

import com.d212.taiso.domain.member.entity.Member;
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

    @ManyToOne(fetch = LAZY)
    @JoinColumn(name = "email")
    private Member member;

    private int cnt = 1; // 경유 인원

    private int order; // 경유 순서

    private boolean depart_flag = false; // 출발지 여부

    private boolean board_flag = false; // 탑승 확인 여부

    private boolean stop_flag = false; // 하차 확인 여부

    private LocalDateTime arrivalTime;

}
