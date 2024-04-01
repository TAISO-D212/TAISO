package com.d212.taiso.domain.member.dto;


import lombok.Builder;
import lombok.Data;

import java.time.LocalDateTime;

@Data
@Builder
public class MemberRes {

    private String email;

    private String name;

    private String faceImg;

    private LocalDateTime createDate;
}
