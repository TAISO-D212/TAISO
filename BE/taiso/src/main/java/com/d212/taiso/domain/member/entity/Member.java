package com.d212.taiso.domain.member.entity;
/**
 * Created by 전근렬 on 2024-03-21
 */

import jakarta.persistence.*;
import lombok.AccessLevel;
import lombok.AllArgsConstructor;
import lombok.Builder;
import lombok.Getter;
import lombok.NoArgsConstructor;
import org.springframework.data.annotation.CreatedDate;
import org.springframework.data.jpa.domain.support.AuditingEntityListener;

import java.time.LocalDateTime;

@Entity
@Getter
@AllArgsConstructor
@NoArgsConstructor(access = AccessLevel.PROTECTED)
@Builder
@Table(name = "member")
//@EntityListeners(AuditingEntityListener.class)
public class Member {

    @Id
    private String email;
    private String pwd;
    private String name; // username으로 쓰이는 것이 있어서 name으로 변환
    private String faceImg;
    private String fcmToken;
    @CreatedDate
    private LocalDateTime createDate;

    @Builder.Default
    private boolean deleteFlag = false;

    public void changePwd(String pwd) {
        this.pwd = pwd;
    }

    public void changeName(String name) {
        this.name = name;
    }

    public void changeFaceImg(String faceImg) {
        this.faceImg = faceImg;
    }

    public void changeDeleteFlag(boolean deleteFlag) {
        this.deleteFlag = deleteFlag;
    }

    public void changeFcmToken(String fcmToken) {
        this.fcmToken = fcmToken;
    }
}

