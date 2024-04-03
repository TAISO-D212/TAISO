import React, { createContext, useContext, ReactNode, useState, useEffect } from 'react';
import mqtt, { MqttClient } from 'mqtt';

interface LocationData {
	latitude: number;
	longitude: number;
}

interface MQTTContextType {
	locationData: LocationData;
	isMsg: boolean;
	setIsMsg: React.Dispatch<React.SetStateAction<boolean>>;
}

const MQTTContext = createContext<MQTTContextType>({
	locationData: { latitude: 37.239227, longitude: 126.773229 },
	isMsg: false,
	setIsMsg: () => {},
});

interface MQTTProviderProps {
	children: ReactNode; // 자식 컴포넌트 타입을 ReactNode로 정의
}

export const useMQTT = () => useContext(MQTTContext);

export const MQTTProvider: React.FC<MQTTProviderProps> = ({ children }) => {
	const [locationData, setLocationData] = useState<LocationData>();
	const [isMsg, setIsMsg] = useState<boolean>(false);

	useEffect(() => {
		const client: MqttClient = mqtt.connect('wss://j10d212.p.ssafy.io:9001', {
			path: '/mqtt',
		});

		client.on('connect', () => {
			console.log('Connected to MQTT Broker');
			client.subscribe('location/FE');
		});

		client.on('message', (topic, message) => {
			if (!!message && !isMsg) {
				setIsMsg(true);
			}
			const msg = message.toString();
			const [latitude, longitude] = msg.split(' ').map(Number);
			const data: LocationData = { latitude, longitude };
			// console.log('data : ', data, 'locationData : ', locationData);
			// setLocationData((prevData) => [...prevData, data]);
			setLocationData(data);
		});

		// MQTT 클라이언트 연결 종료
		return () => {
			client.end();
		};
	}, []);

	// children을 반환하여 자식 컴포넌트를 래핑
	return (
		<MQTTContext.Provider value={{ locationData, isMsg, setIsMsg }}>{children}</MQTTContext.Provider>
	);
};
