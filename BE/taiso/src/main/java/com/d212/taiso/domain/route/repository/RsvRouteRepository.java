package com.d212.taiso.domain.route.repository;
/**
 * Created by 배성연 on 2024-03-21
 */

import com.d212.taiso.domain.route.entity.RsvRoute;
import org.springframework.data.jpa.repository.JpaRepository;

public interface RsvRouteRepository extends JpaRepository<RsvRoute, Integer> {
    // 여기에 추가적인 쿼리 메소드 정의

}
