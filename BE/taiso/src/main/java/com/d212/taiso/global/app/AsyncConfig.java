package com.d212.taiso.global.app;

import org.springframework.context.annotation.Bean;
import org.springframework.context.annotation.Configuration;
import org.springframework.scheduling.annotation.EnableAsync;
import org.springframework.scheduling.concurrent.ThreadPoolTaskExecutor;

/**
 * Created by 배성연 on 2024-03-21
 */

@Configuration
@EnableAsync
public class AsyncConfig {
    // 비동기 실행 활성화

    @Bean
    public ThreadPoolTaskExecutor taskExecutor() {
        // mqtt 요청 동시 처리를 위한 thread pool 설정
        ThreadPoolTaskExecutor executor = new ThreadPoolTaskExecutor();
        executor.setCorePoolSize(4);    // 기본 스레드 수
        executor.setMaxPoolSize(20);    // 최대 스레드 수
        executor.setQueueCapacity(100); //큐 용량
        executor.setThreadNamePrefix("MQTTExecutor-");
        executor.initialize();
        return executor;
    }
}

