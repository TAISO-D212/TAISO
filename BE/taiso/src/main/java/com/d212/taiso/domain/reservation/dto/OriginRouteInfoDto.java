package com.d212.taiso.domain.reservation.dto;

import lombok.AllArgsConstructor;
import lombok.Builder;
import lombok.Data;

/**
 * Created by 배성연 on 2024-03-28
 */
@Data
@Builder
@AllArgsConstructor
public class OriginRouteInfoDto {

    private String routeDist;
    private int stopCnt;
}
