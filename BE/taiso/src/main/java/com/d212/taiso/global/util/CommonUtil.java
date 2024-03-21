package com.d212.taiso.global.util;
/**
 * Created by 전근렬 on 2024-03-21
 */

import com.d212.taiso.domain.member.entity.Member;
import com.d212.taiso.domain.member.repository.MemberRepository;
import com.d212.taiso.global.result.error.ErrorCode;
import com.d212.taiso.global.result.error.exception.BusinessException;
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
            .orElseThrow(() -> new BusinessException(ErrorCode.MEMBER_EMAIL_NOT_EXIST));

        // 회원탈퇴여부 체크
        if (currentMember.isDeleteFlag()) {
            throw new BusinessException(ErrorCode.MEMBER_DISABLED);
        }

        return currentMember;
    }
}
