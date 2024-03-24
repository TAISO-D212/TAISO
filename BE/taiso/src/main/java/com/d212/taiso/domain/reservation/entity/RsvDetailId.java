package com.d212.taiso.domain.reservation.entity;

import static jakarta.persistence.FetchType.*;

import com.d212.taiso.domain.place.entity.Place;
import com.d212.taiso.domain.reservation.entity.Reservation;
import jakarta.persistence.Embeddable;
import jakarta.persistence.FetchType;
import jakarta.persistence.JoinColumn;
import jakarta.persistence.ManyToOne;
import jakarta.persistence.MapsId;
import java.io.Serializable;
import lombok.AllArgsConstructor;
import lombok.Data;
import lombok.NoArgsConstructor;

/**
 * Created by 전근렬 on 2024-03-21
 */
// 복합키
@Data
@Embeddable
@NoArgsConstructor
public class RsvDetailId implements Serializable {

    // 복합키 방식 2
//    private Long rsvId;
//    private Long placeId;

    @ManyToOne(fetch = LAZY)
    @JoinColumn(name = "rsv_id", nullable = false)
    private Reservation reservation;

    @ManyToOne(fetch = LAZY)
    @JoinColumn(name = "place_id", nullable = false)
    private Place place;


}
