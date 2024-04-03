package com.d212.taiso.domain.member.service;

import com.d212.taiso.domain.member.dto.AlertDto;
import com.google.firebase.messaging.FirebaseMessagingException;
import org.hibernate.sql.exec.ExecutionException;

public interface AlertService {

    public void sendAlert(AlertDto alert) throws ExecutionException, InterruptedException, FirebaseMessagingException;

    public void departAlertSend(Long rsvId) throws ExecutionException, InterruptedException, FirebaseMessagingException;

    public void soonAlertSend(Long rsvId) throws ExecutionException, InterruptedException, FirebaseMessagingException;

    public void arrivalAlertSend(Long rsvId) throws ExecutionException, InterruptedException, FirebaseMessagingException;

//    public String getNotificationToken();
}
