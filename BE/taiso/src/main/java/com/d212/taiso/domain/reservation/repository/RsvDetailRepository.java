package com.d212.taiso.domain.reservation.repository;

import com.d212.taiso.domain.reservation.entity.RsvDetail;
import com.d212.taiso.domain.reservation.entity.RsvDetailId;
import org.springframework.data.jpa.repository.JpaRepository;

public interface RsvDetailRepository extends JpaRepository<RsvDetail, RsvDetailId> {

}
