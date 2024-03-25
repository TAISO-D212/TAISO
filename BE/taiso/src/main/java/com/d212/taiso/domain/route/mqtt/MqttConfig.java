package com.d212.taiso.domain.route.mqtt;

import java.util.UUID;
import lombok.extern.log4j.Log4j2;
import org.eclipse.paho.client.mqttv3.MqttConnectOptions;
import org.springframework.context.annotation.Bean;
import org.springframework.context.annotation.Configuration;
import org.springframework.beans.factory.annotation.Value;
import org.springframework.integration.annotation.IntegrationComponentScan;
import org.springframework.integration.channel.DirectChannel;
import org.springframework.integration.dsl.IntegrationFlow;
import org.springframework.integration.mqtt.core.DefaultMqttPahoClientFactory;
import org.springframework.integration.mqtt.core.MqttPahoClientFactory;
import org.springframework.integration.mqtt.inbound.MqttPahoMessageDrivenChannelAdapter;
import org.springframework.integration.mqtt.outbound.MqttPahoMessageHandler;
import org.springframework.integration.mqtt.support.DefaultPahoMessageConverter;
import org.springframework.messaging.MessageChannel;
import org.springframework.messaging.MessageHandler;

/**
 * Created by 배성연 on 2024-03-21
 */

@Log4j2
@Configuration
@IntegrationComponentScan
public class MqttConfig {

    @Value("${mqtt.broker.url}")
    private String brokerUrl;

    private String outClientId = UUID.randomUUID().toString();
    private String inClientId = UUID.randomUUID().toString();

    @Bean
    public MqttPahoClientFactory mqttPahoClientFactory() {
        DefaultMqttPahoClientFactory factory = new DefaultMqttPahoClientFactory();
        MqttConnectOptions connectOptions = new MqttConnectOptions();
        connectOptions.setServerURIs(new String[]{brokerUrl});
        factory.setConnectionOptions(connectOptions);
        return factory;
    }

    @Bean
    public MessageChannel mqttInputChannel() {
        return new DirectChannel();
    }

    @Bean
    public MessageChannel mqttOutputChannel() {
        return new DirectChannel();
    }

    @Bean
    public MqttPahoMessageDrivenChannelAdapter mqttInboundAdapter() {
        MqttPahoMessageDrivenChannelAdapter adapter = new MqttPahoMessageDrivenChannelAdapter(
            brokerUrl, inClientId);
        adapter.addTopic("distance/EMB", 1);
        adapter.addTopic("location/BE", 1);
        adapter.setCompletionTimeout(5000);
        adapter.setConverter(new DefaultPahoMessageConverter());
        adapter.setQos(1);
        log.info("InBound 메시지 채널 Adapter 생성 완료");
        return adapter;
    }

    @Bean
    public MqttPahoMessageHandler mqttOutboundHandler() {
        MqttPahoMessageHandler messageHandler = new MqttPahoMessageHandler(
            outClientId,
            mqttPahoClientFactory());
        messageHandler.setAsync(true);
        messageHandler.setDefaultTopic("noTopic");
        log.info("OutBound 메시지 핸들러 생성 완료");
        return messageHandler;
    }

    @Bean
    public IntegrationFlow mqttInboundFlow() {
        return IntegrationFlow.from(mqttInboundAdapter())
            .channel(mqttInputChannel())
            .get();
    }

    @Bean
    public IntegrationFlow mqttOutboundFlow() {
        return IntegrationFlow.from(mqttOutputChannel())
            .handle(mqttOutboundHandler())
            .get();
    }

}
