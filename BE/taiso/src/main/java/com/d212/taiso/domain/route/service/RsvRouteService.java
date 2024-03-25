package com.d212.taiso.domain.route.service;
/**
 * Created by 배성연 on 2024-03-21
 */

import com.d212.taiso.domain.route.entity.RsvRoute;

public interface RsvRouteService {

    void locationToRoute(long rsvId, long placeId);

}
