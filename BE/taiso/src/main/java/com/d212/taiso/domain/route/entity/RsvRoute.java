package com.d212.taiso.domain.route.entity;
/**
 * Created by 배성연 on 2024-03-21
 */

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
    @Id
    private int rsvId;
    private double latitude;
    private double longitude;
    private LocalDateTime time;
}
