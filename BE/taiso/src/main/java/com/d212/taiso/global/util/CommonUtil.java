package com.d212.taiso.global.util;
import com.d212.taiso.domain.member.entity.Member;
import com.d212.taiso.domain.member.repository.MemberRepository;
import com.d212.taiso.global.exception.member.MemberDisabledException;
import com.d212.taiso.global.exception.member.MemberNotFoundException;
import lombok.RequiredArgsConstructor;
import org.springframework.security.core.Authentication;
import org.springframework.security.core.context.SecurityContextHolder;
import org.springframework.stereotype.Component;

@Component
@RequiredArgsConstructor
public class CommonUtil {

    private final MemberRepository memberRepository;

    // 토큰으로부터 회원의 정보를 가져오고 탈퇴여부까지 확인을 해줌.
    public Member getMember() {
        Authentication authentication = SecurityContextHolder.getContext().getAuthentication();
        String userEmail = authentication.getName();
        Member currentMember = memberRepository.findMemberByEmail(userEmail)
                .orElseThrow(MemberNotFoundException::new);

        // 회원탈퇴여부 체크
        if (currentMember.isDeleteFlag()) {
            throw new MemberDisabledException();
        }

        return currentMember;
    }
}
