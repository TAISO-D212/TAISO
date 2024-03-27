package com.d212.taiso.domain.route.dto;

import lombok.Data;
import lombok.Builder;
import lombok.AllArgsConstructor;

/**
 * Created by 배성연 on 2024-03-21
 */

@Data
@Builder
@AllArgsConstructor
public class LocationDto {

    private long placeId;
    private double latitude;
    private double longitude;

}
