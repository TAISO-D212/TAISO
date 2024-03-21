package com.d212.taiso.domain.member.dto;
/**
 * Created by 전근렬 on 2024-03-21
 */

import lombok.Getter;
import lombok.Setter;
import lombok.ToString;
import org.springframework.security.core.userdetails.User;

import java.time.LocalDateTime;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.Map;

@Getter
@Setter
@ToString
public class MemberDTO extends User {

    private String email, pwd, name, faceImg;

    private boolean deleteFlag;

    // 생성날짜도 넣어야 하나?
    private LocalDateTime createDate;

    // todo 보류
    public MemberDTO(String email, String pwd, String name, boolean deleteFlag) {
        super(email, pwd, new ArrayList<>());
        this.email = email;
        this.pwd = pwd;
        this.name = name;
        this.deleteFlag = deleteFlag;
    }

    // 이 객체를 주고 받는 것이 아니라 JWT라는 문자열을 만들어서 주고 받을 것인데 JWT 문자를 만들 때 이 데이터가 필요
    // 이 데이터를 바꿔가지고 뭔가를 처리하게 되는데 그 처리를 위해서 JWT 문자열에 내용물을 claims라고 한다.

    public Map<String, Object> getClaims() {
        Map<String, Object> dataMap = new HashMap<>();
        dataMap.put("email", email);
        dataMap.put("pwd", pwd);
        dataMap.put("name", name);
        dataMap.put("deleteFlag", deleteFlag);
        return dataMap;
    }
}
