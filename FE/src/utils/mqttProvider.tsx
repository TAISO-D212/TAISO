import React, { createContext, useContext, ReactNode, useState, useEffect } from 'react';
import mqtt, { MqttClient } from 'mqtt';

interface LocationData {
  lat: number;
  lon: number;
}

interface MQTTContextType {
  locationData: LocationData[];
}

const MQTTContext = createContext<MQTTContextType>({ locationData: [] });

interface MQTTProviderProps {
  children: ReactNode; // 자식 컴포넌트 타입을 ReactNode로 정의
}

export const useMQTT = () => useContext(MQTTContext);

export const MQTTProvider: React.FC<MQTTProviderProps> = ({ children }) => {
  const [locationData, setLocationData] = useState<LocationData[]>([]);

  useEffect(() => {
    const client: MqttClient = mqtt.connect('ws://j10d212.p.ssafy.io:9001', {
      path: '/mqtt',
    });

    client.on('connect', () => {
      console.log("Connected to MQTT Broker");
      client.subscribe("location/FE");
    });

    client.on('message', (topic, message) => {
      const msg = message.toString();
      const [lat, lon] = msg.split(' ').map(Number);
      const data: LocationData = {lat, lon};
      setLocationData(prevData => [...prevData, data]);
    });

    // MQTT 클라이언트 연결 종료
    return () => {
      client.end();
    };
  }, []);

  // children을 반환하여 자식 컴포넌트를 래핑
  return <MQTTContext.Provider value={{ locationData }}>
    {children}
  </MQTTContext.Provider>;
};
