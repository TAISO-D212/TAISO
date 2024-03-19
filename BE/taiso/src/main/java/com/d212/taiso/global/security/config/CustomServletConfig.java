package com.d212.taiso.global.security.config;

import com.d212.taiso.domain.member.controller.formatter.LocalDateFormatter;
import lombok.extern.log4j.Log4j2;
import org.springframework.context.annotation.Configuration;
import org.springframework.format.FormatterRegistry;
import org.springframework.web.servlet.config.annotation.WebMvcConfigurer;

@Configuration
@Log4j2
public class CustomServletConfig implements WebMvcConfigurer {
    @Override
    public void addFormatters(FormatterRegistry registry) {

        log.info("-----------------------------------");
        log.info("addFormatters");

        registry.addFormatter(new LocalDateFormatter());
    }
}

