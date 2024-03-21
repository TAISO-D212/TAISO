package com.d212.taiso.global.result;
/**
 * Created by 전근렬 on 2024-03-21
 */

import lombok.Getter;

@Getter
public class ResultResponse {

    private final int status;
    private final String message;
    private final Object data;

    private ResultResponse(ResultCode resultCode, Object data) {
        this.status = resultCode.getStatus();
        this.message = resultCode.getMessage();
        this.data = data;
    }

    public static ResultResponse of(ResultCode resultCode, Object data) {
        return new ResultResponse(resultCode, data);
    }

    public static ResultResponse of(ResultCode resultCode) {
        return new ResultResponse(resultCode, "");
    }

}
