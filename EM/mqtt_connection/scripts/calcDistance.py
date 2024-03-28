#!/usr/bin/python2.7
# -*- coding: utf-8 -*-

import sys
import math
import json
import paho.mqtt.publish as publish
from pyproj import Proj, transform
from path_length import PathLengthCalculator

def calc_distance(locations):
    if not locations:
        return []
    
    calculator = PathLengthCalculator()

    # 마지막 위치 기준으로 계산
    distanceList = []
    base_location = locations[-1]
    for location in locations[:-1]:
        distance = distance_between(base_location, location, calculator)
        distanceList.append(distance)
    print("Calculated Distance List : ", distanceList)
    return distanceList

def distance_between(loc1, loc2, calculator):
    # 다익스트라, 맵 데이터 이용 거리 계산 로직

    # gps -> utm 좌표 변환
    wgs84 = Proj(init='epsg:4326')
    utm_proj = Proj(proj='utm', zone=52, ellps='WGS84', preserve_units=False)
    utm_x1, utm_y1 = transform(wgs84, utm_proj, loc1['longitude'], loc1['latitude'])
    utm_x2, utm_y2 = transform(wgs84, utm_proj, loc2['longitude'], loc2['latitude'])
    utm_x1 -= 302459.942
    utm_y1 -= 4122635.537
    utm_x2 -= 302459.942
    utm_y2 -= 4122635.537

    print('utm_x1 : {}, utm_y1 : {}'.format(utm_x1, utm_y1))
    print('utm_x2 : {}, utm_y2 : {}'.format(utm_x2, utm_y2))

    # 위경도 사용, 가장 가까운 노드 인덱스 찾기
    start_node_idx = find_nearest_node_index(utm_x1, utm_y1, calculator.nodes)
    end_node_idx = find_nearest_node_index(utm_x2, utm_y2, calculator.nodes)
    print('start_node_idx : {}, end_node_idx : {}'.format(start_node_idx, end_node_idx))

    distance = calculator.find_shortest_path_distance(start_node_idx, end_node_idx)
    return distance

def find_nearest_node_index(x, y, nodes):
    nearest_node_idx = None
    min_dist = float('inf')
    for node_idx, node in nodes.iteritems():
        node_x, node_y = node.point[0], node.point[1]
        dist = math.sqrt((x - node_x)**2 + (y - node_y)**2)
        if dist < min_dist:
            min_dist = dist
            nearest_node_idx = node_idx
    return nearest_node_idx


if __name__ == "__main__":
    print("calcDistance running=========")
    if len(sys.argv) >1:
        json_data = sys.argv[1]
        data = json.loads(json_data)
        
        placeId = data.get('placeId')
        locations = data.get('locations', [])
        rsvId = data.get('rsvId')

        # 파싱된 데이터 출력 (테스트용)
        print('Place ID:', placeId)
        print('Locations:')
        for loc in locations:
            print('  Place ID: {0}, Latitude: {1}, Longitude: {2}'.format(loc['placeId'], loc['latitude'], loc['longitude']))
        print('Reservation ID:', rsvId)

        # 거리 계산
        distanceList = calc_distance(locations)

        # 발행할 데이터 생성
        publish_data = json.dumps({
            'placeId': placeId,
            'rsvId': rsvId,
            'distanceList': distanceList
        })

        # mqtt 발행
        publish.single("distance/EMB", publish_data, hostname="j10d212.p.ssafy.io")
        print("data publish completed : ", publish_data)

    else:
        print("error : No data!")

    
