package com.d212.taiso.domain.rsvdetail.entity;

import static jakarta.persistence.FetchType.*;

import com.d212.taiso.domain.place.entity.Place;
import com.d212.taiso.domain.reservation.entity.Reservation;
import jakarta.persistence.Embeddable;
import jakarta.persistence.FetchType;
import jakarta.persistence.JoinColumn;
import jakarta.persistence.ManyToOne;
import java.io.Serializable;
import lombok.Data;

/**
 * Created by 전근렬 on 2024-03-21
 */
// 복합키
// (EAGER로 한다???)
@Data
@Embeddable
public class RsvDetailId implements Serializable {

    @ManyToOne(fetch = LAZY)
    @JoinColumn(name = "rsv_id")
    private Reservation reservation;

    @ManyToOne(fetch = LAZY)
    @JoinColumn(name = "place_id")
    private Place place;
}
