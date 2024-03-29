package com.d212.taiso.domain.member.service;

import com.google.firebase.FirebaseApp;
import com.google.firebase.FirebaseOptions;
import jakarta.annotation.PostConstruct;
import lombok.extern.slf4j.Slf4j;
import org.springframework.core.io.ClassPathResource;
import org.springframework.stereotype.Service;

import java.io.IOException;
import com.google.auth.oauth2.GoogleCredentials;


/* FCM을 실행시키는 코드 -> 초기 준비 단계 */
@Service
@Slf4j
public class FCMInitializer {


//    ClassPathResource()는 resources 폴더 아래에 있는 괄호 안 경로를 찾는다.
//     json파일을 찾아서 맞는 정보인지 확인한 후 FirebaseApp.initializeApp()을 통해서 실행한다.
//     서버 실행시에 딱 한번 실행되어야 하므로 @PostConstruct 어노테이션을 붙인다.
    private static final String FIREBASE_CONFIG_PATH = "taiso-18ea8-firebase-adminsdk-m9qcd-e458ff02c9.json";

    @PostConstruct
    public void initialize() {
        try {
            GoogleCredentials googleCredentials = GoogleCredentials
                    .fromStream(new ClassPathResource(FIREBASE_CONFIG_PATH).getInputStream());
            FirebaseOptions options = new FirebaseOptions.Builder()
                    .setCredentials(googleCredentials)
                    .build();
            FirebaseApp.initializeApp(options);
        } catch (IOException e) {
            log.info(">>>>>>>>FCM error");
            log.error(">>>>>>FCM error message : " + e.getMessage());
        }
    }
}

//FirebaseOptions options = FirebaseOptions.builder()
//        .setCredentials(GoogleCredentials.getApplicationDefault())
//        .setDatabaseUrl("https://<DATABASE_NAME>.firebaseio.com/")
//        .build();
//
//FirebaseApp.initializeApp(options);