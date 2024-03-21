package com.d212.taiso.global.security.util;

/**
 * Created by 전근렬 on 2024-03-21
 */
public class CustomJWTException extends RuntimeException {

    public CustomJWTException(String msg) {
        super(msg);
    }
}

