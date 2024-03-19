package com.d212.taiso.domain.member.entity;

import jakarta.persistence.*;
import lombok.AllArgsConstructor;
import lombok.Builder;
import lombok.Getter;
import lombok.NoArgsConstructor;
import org.springframework.data.annotation.CreatedDate;

import java.time.LocalDateTime;

@Entity
@Getter
@AllArgsConstructor
@NoArgsConstructor
@Builder
public class Member {

    @Id
    private String email;
    private String pw;
    private String name; // username으로 쓰이는 것이 있어서 name으로 변환
    private String faceImg;
    @CreatedDate
    private LocalDateTime createTime;
    private boolean deleteFlag;
    public void changePw(String pw) {this.pw = pw;}
    public void changeName(String name) {this.name = name;}
    public void changeFaceImg(String faceImg) {this.faceImg = faceImg;}
    public void changeDeleteFlag(boolean deleteFlag) {this.deleteFlag = deleteFlag;}

}

