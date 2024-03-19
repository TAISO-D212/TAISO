package com.d212.taiso.global.security.util;

public class CustomJWTException extends RuntimeException{
    public CustomJWTException(String msg){
        super(msg);
    }
}

