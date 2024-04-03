package com.d212.taiso.domain.route.entity;
/**
 * Created by 배성연 on 2024-03-21
 */

import static jakarta.persistence.FetchType.*;

import com.d212.taiso.domain.reservation.entity.Reservation;
import jakarta.persistence.*;
import lombok.AllArgsConstructor;
import lombok.Builder;
import lombok.Getter;
import lombok.NoArgsConstructor;

import java.time.LocalDateTime;

@Entity
@Getter
@AllArgsConstructor
@NoArgsConstructor
@Builder
public class RsvRoute {

    // 외래키이자 pk가 되도록 설정
    @Id
    @Column(name = "route_id", nullable = false)
    private Long id;

    @ManyToOne(fetch = LAZY)
    @JoinColumn(name = "rsv_id")
    private Reservation reservation;

    private double latitude;
    private double longitude;
    private LocalDateTime time;
}
