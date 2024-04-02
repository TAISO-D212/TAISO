package com.d212.taiso;

import io.github.cdimascio.dotenv.Dotenv;
import org.springframework.boot.SpringApplication;
import org.springframework.boot.autoconfigure.SpringBootApplication;
import org.springframework.scheduling.annotation.EnableScheduling;

@EnableScheduling
@SpringBootApplication
public class TaisoApplication {

    public static void main(String[] args) {

        Dotenv dotenv = Dotenv.configure()
            .directory("./")
            .load();
        dotenv.entries().forEach(entry -> {
            System.setProperty(entry.getKey(), entry.getValue());
        });

        SpringApplication.run(TaisoApplication.class, args);
    }

}
