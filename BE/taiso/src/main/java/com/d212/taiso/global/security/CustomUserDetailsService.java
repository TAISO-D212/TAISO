package com.d212.taiso.global.security;
/**
 * Created by 전근렬 on 2024-03-21
 */

import com.d212.taiso.domain.member.dto.MemberDTO;
import com.d212.taiso.domain.member.entity.Member;
import com.d212.taiso.domain.member.repository.MemberRepository;
import lombok.RequiredArgsConstructor;
import lombok.extern.log4j.Log4j2;
import org.springframework.security.core.userdetails.UserDetails;
import org.springframework.security.core.userdetails.UserDetailsService;
import org.springframework.security.core.userdetails.UsernameNotFoundException;
import org.springframework.stereotype.Service;

// 잘된 자바 코드들은 항상 리턴 타입이랑 파라미터 타입은 가능하면 인터페이스로 뺌.(나중에 변경이 용이 하도록), 늘 김영한이 하던거네.
// 현재 리턴 타입이 유저 Details 타입인데, 근데 얘를 구현한 클래스 중에 하나가 유저(유저는 클래스이다보니 생성자를 가질 수 있음.)
// 그냥 어지럽네.

@RequiredArgsConstructor
@Service
@Log4j2
public class CustomUserDetailsService implements UserDetailsService {

    // 여기서 유저네임은 우리에게 ID에 해당하는 값이다.
    // 리턴 타입을 보면 UserDetails임을 알 수 있다. -> 즉 우리에게는 이게 멤버 DTO가 됨을 알 수 있다.
    // 얘는 인터페이스 이고 유저라는 애(Member DTO 쪽에 USER라는 애를 extends 했음)는 클래스 느낌....

    private final MemberRepository memberRepository;

    @Override
    public UserDetails loadUserByUsername(String username) throws
        UsernameNotFoundException {
        log.info("----------------loadUserByUsername----------------------");

        // 여기서 username이 이메일임
        Member member = memberRepository.findMemberByEmail(username)
            .orElseThrow();

        // 이렇게 뽑아낸 Member 엔티티를 가지고 멤버DTO를 반환해줘야 됨

        if (member == null) {
            throw new UsernameNotFoundException("Not Found");
        }

        MemberDTO memberDTO = new MemberDTO(
            member.getEmail(),
            member.getPwd(),
            member.getName(),
            member.isDeleteFlag());

        log.info(memberDTO);

        return memberDTO;

    }

}

