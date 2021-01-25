'''该类为节点的自扩展与自吸纳方法类
    节点自扩展：节点将附近通信范围内的节点特征值进行比较，相近则纳为一个集群里，从而选择一个影响力最大化节点
    节点自吸纳：在经过分叉路口时节点可能会脱离原集群，此时需重新为其寻找影响力最大化节点
'''
import numpy as np
import Global_Par as glp


def distance(position1, position2):
    dist = np.sqrt(sum((np.array(position1) - np.array(position2)) ** 2))
    return dist


def direction(self, direction1, direction2):
    re = False
    if direction1 == direction2:
        re = True
    return re


def gvlaue(node1, RG):
    weight = 0
    for node1, node2 in RG.edges(node1):    # node1 的度与权重
        weight += RG[node1][node2]['weight']
    D = RG.degree(node1)
    return (D / 2) / weight


def gvlaue2(node):
    weight = 0
    # print('--------')
    for neighbor in node.IMN:    # node1 的度与权重
        # print(neighbor.node_id)
        # print(neighbor.position, node.position, node.IMN_LIST)
        weight += distance(neighbor.position, node.position)
    D = len(node.IMN)
    return (D / 2) / weight


class ExpAbs:

    def distance(self, position1, position2):
        dist = np.sqrt(sum((np.array(position1)-np.array(position2))**2))
        return dist

    def direction(self, direction1, direction2):
        re = False
        if direction1 == direction2:
            re = True
        return re

    def com_distance(self, node_list, com_node_list):
        # 计算通信节点列表中源节点与目的节点距离的列表
        com_distance = []
        # print(com_node_list)
        for i in range(len(com_node_list)):
            com_distance.append(
                self.distance(node_list[com_node_list[:][i][0]].position, node_list[com_node_list[:][i][1]].position))

        return com_distance

