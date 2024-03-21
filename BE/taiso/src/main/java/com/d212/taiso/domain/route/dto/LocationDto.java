package com.d212.taiso.domain.route.dto;

import lombok.AllArgsConstructor;
import lombok.Data;
import lombok.Getter;
import lombok.NoArgsConstructor;
import lombok.Setter;
import lombok.ToString;

/**
 * Created by 배성연 on 2024-03-21
 */

@Data
@Getter
@Setter
@ToString
@NoArgsConstructor
@AllArgsConstructor
public class LocationDto {

    private double latitude;
    private double longitude;
    private String name;

}
