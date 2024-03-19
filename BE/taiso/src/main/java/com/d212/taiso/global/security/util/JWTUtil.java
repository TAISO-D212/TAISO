package com.d212.taiso.global.security.util;

import io.jsonwebtoken.*;
import io.jsonwebtoken.security.Keys;
import lombok.extern.log4j.Log4j2;

import javax.crypto.SecretKey;
import java.time.ZonedDateTime;
import java.util.Date;
import java.util.Map;

@Log4j2
public class JWTUtil {
    // key 값을 딴거 쓰고 싶을 수 있는데, 너무 짧으면 안된다. 30자 이상으로 할 것.
    // 또 이걸 프로퍼티 파일? 로 빼서 관리를 하는 법도 있긴한데(스프링 빈으로 관리) 난 안해

    // 그리고 여기서 하는 건 아니긴 한데 나중에 payload에서 pw같은 민감한 정보는 뺄 것.
    private static String key = "1234567890123456789012345678901234567890";

    // JWT 문자열 생성을 위한 함수
    public static String generateToken(Map<String, Object> valueMap, int min) {
        SecretKey key = null;
        try {
            key = Keys.hmacShaKeyFor(JWTUtil.key.getBytes("UTF-8"));
        } catch (Exception e) {
            throw new RuntimeException(e.getMessage());
        }
        String jwtStr = Jwts.builder()
                .setHeader(Map.of("typ", "JWT"))
                .setClaims(valueMap)
                .setIssuedAt(Date.from(ZonedDateTime.now().toInstant()))
                .setExpiration(Date.from(ZonedDateTime.now().plusMinutes(min).toInstant()))
                .signWith(key)
                .compact();
        return jwtStr;
    }

    // 검증을 위한 validateToken
    public static Map<String, Object> validateToken(String token) {
        Map<String, Object> claim = null;
        try{
            SecretKey key = Keys.hmacShaKeyFor(JWTUtil.key.getBytes("UTF-8"));
            claim = Jwts.parserBuilder()
                    .setSigningKey(key)
                    .build()
                    .parseClaimsJws(token) // 파싱 및 검증, 실패 시 에러
                    .getBody();
        }catch(MalformedJwtException malformedJwtException){
            throw new CustomJWTException("MalFormed");
        }catch(ExpiredJwtException expiredJwtException){
            throw new CustomJWTException("Expired");
        }catch(InvalidClaimException invalidClaimException){
            throw new CustomJWTException("Invalid");
        }catch(JwtException jwtException){
            throw new CustomJWTException("JWTError");
        }catch(Exception e){
            throw new CustomJWTException("Error");
        }
        return claim;
    }
}