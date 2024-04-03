package com.d212.taiso.domain.member.service;

import com.d212.taiso.domain.member.dto.AlertDto;
import org.hibernate.sql.exec.ExecutionException;

public interface AlertService {

    public void sendAlert(AlertDto alert) throws ExecutionException, InterruptedException;

    public void departAlertSend(Long rsvId) throws ExecutionException, InterruptedException;

    public void soonAlertSend(Long rsvId) throws ExecutionException, InterruptedException;

    public void arrivalAlertSend(Long rsvId) throws ExecutionException, InterruptedException;

//    public String getNotificationToken();
}
