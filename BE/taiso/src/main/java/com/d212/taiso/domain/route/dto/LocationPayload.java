package com.d212.taiso.domain.route.dto;

import java.util.List;
import lombok.Data;
import lombok.Builder;
import lombok.NoArgsConstructor;
import lombok.AllArgsConstructor;

/**
 * Created by 배성연 on 2024-03-28
 */

@Data
@Builder
@NoArgsConstructor
@AllArgsConstructor
public class LocationPayload {

    private List<Long> distanceList;
    private long placeId;
    private long rsvId;
}
