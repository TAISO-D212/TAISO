#!/usr/bin/python2.7
# -*- coding: utf-8 -*-

import os
import sys
import json
import numpy as np
from math import sqrt

current_path = os.path.dirname(os.path.realpath(__file__))
sys.path.append(current_path)

from lib.mgeo.class_defs import *

class PathLengthCalculator :
    def __init__(self):
        load_path = os.path.normpath(os.path.join(current_path, 'lib/mgeo_data/R_KR_PG_K-City'))
        mgeo_planner_map = MGeo.create_instance_from_json(load_path)

        self.nodes = mgeo_planner_map.node_set.nodes
        self.links = mgeo_planner_map.link_set.lines
        self.weight = self.get_weight_matrix()

    def get_weight_matrix(self):
         # 초기 설정
        weight = {}
        for from_node_id, from_node in self.nodes.items():
            # 현재 노드에서 다른 노드로 진행하는 모든 weight
            weight_from_this_node = {to_node_id: float('inf') for to_node_id in self.nodes.keys()}
            # 전체 weight matrix에 추가
            weight[from_node_id] = weight_from_this_node

        for from_node_id, from_node in self.nodes.items():
            # 현재 노드에서 현재 노드로는 cost = 0
            weight[from_node_id][from_node_id] = 0

            for to_node in from_node.get_to_nodes():
                # 현재 노드에서 to_node로 연결되어 있는 링크를 찾고, 그 중에서 가장 빠른 링크를 찾아준다
                shortest_link, min_cost = self.find_shortest_link_leading_to_node(from_node,to_node)
                if shortest_link:
                    weight[from_node_id][to_node.idx] = min_cost

        return weight
    
    def find_shortest_link_leading_to_node(self, from_node,to_node):
        shortest_link = None
        min_cost = float('inf')

        for link in from_node.get_to_links():
            if link.to_node is to_node and link.cost < min_cost:
                min_cost = link.cost
                shortest_link = link
        return shortest_link, min_cost

    def calculate_distance_between_nodes(self, node1, node2):
        x1, y1 = node1.point[0], node1.point[1]
        x2, y2 = node2.point[0], node2.point[1]
        distance = sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)
        return distance

    def find_shortest_path_distance(self, start_node_idx, end_node_idx):
        distance = {node_idx: float('inf') for node_idx in self.nodes}
        distance[start_node_idx] = 0
        visited = set()
        while len(visited) < len(self.nodes):
            current_node_idx = min((node for node in self.nodes if node not in visited), key = lambda node: distance[node])
            visited.add(current_node_idx)
            current_node = self.nodes[current_node_idx]
            for to_node in current_node.get_to_nodes():
                if to_node.idx not in visited:
                    tentative_distance = distance[current_node_idx] + self.calculate_distance_between_nodes(current_node, to_node)
                    if tentative_distance < distance[to_node.idx]:
                        distance[to_node.idx] = tentative_distance
        return distance[end_node_idx]


if __name__ == '__main__':
    calculator = PathLengthCalculator()
    start_node_idx = 'N1'  # 예시 값, 실제 노드 인덱스로 대체 필요
    end_node_idx = 'N10'  # 예시 값, 실제 노드 인덱스로 대체 필요
    total_distance = calculator.find_shortest_path_distance(start_node_idx, end_node_idx)
    print "Total distance between %s and %s is: %s meters" % (start_node_idx, end_node_idx, total_distance)