package com.d212.taiso.domain.member.service;

import com.d212.taiso.domain.member.dto.AlertDto;

public interface AlertService {
    public void sendAlert(AlertDto alert);

}
