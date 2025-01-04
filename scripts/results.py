#!/usr/bin/env python3
# -*- coding: utf-8 -*-
## If you want to try all valid start and goals in your map and save it on a pandas dataframe ##

import rospy
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped,Point
import numpy as np
import random
import pandas as pd
import matplotlib.pyplot as plt
import math
import json
import rospkg
import heapq
from tf.transformations import quaternion_from_euler
import sys


OCCUPIED_THRESHOLD = 50  # Valor mínimo para considerar uma célula ocupada
BUFFER_RADIUS = 1 # Células ao redor de obstáculos tratadas como ocupadas

class node:
  def __init__(self,pose,peso):
    self.pose = pose
    self.peso = peso
    self.parent = [] ## o parent agora e uma lista
    self.dist = None

def init_nodes(field,init_peso=float('inf')):
  nodes = {}
  all_nodes = {}
  for x_i in range(field.shape[0]):
    for y_i in range(field.shape[1]):
      if field[x_i,y_i] == 0:
        all_nodes[x_i,y_i] =  node(pose=[x_i,y_i],peso=init_peso)

        actual  = node(pose=[x_i,y_i],peso=init_peso)

        nodes[actual.pose[0],actual.pose[1]] = actual


  return all_nodes,nodes

def construct_graph(field,grid=1,mov=8):
  graph = {}

  if mov ==4:
    directions = [[grid,0],[-grid,0],[0,grid],[0,-grid]]
  elif mov ==8:
    directions = [[grid,0],[-grid,0],[0,grid],[0,-grid],[-grid,grid],[grid,grid],[grid,-grid],[-grid,-grid]]
  else:
    print("Mov deve ser igual a 4 ou 8")
    return

  for x_i in range(field.shape[0]):
    for y_i in range(field.shape[1]):
      if field[x_i,y_i] == 0:
        graph[x_i,y_i] = []

        for dire in directions:

          possible = np.array([x_i,y_i]) + np.array(dire)

          if possible[0] >= field.shape[0] or possible[1] >= field.shape[1]:
            continue

          if field[possible[0],possible[1]] == 0 and possible[0] >= 0 and possible[1] >= 0 and possible[0] <= field.shape[0]  and possible[1] <= field.shape[1] :

            graph[x_i,y_i].append(list(possible))

  return graph

def wavefront(nodes,goal,graph):
  visited_nodes = []
  nodes[goal[0],goal[1]].peso = 0

  visited_nodes.append(goal)

  actual_node = goal

  for actual_node in visited_nodes:
    for vizinho in graph[actual_node[0],actual_node[1]]:

      if vizinho not in visited_nodes :

        nodes[vizinho[0],vizinho[1]].peso = nodes[actual_node[0],actual_node[1]].peso +1

        visited_nodes.append(vizinho)

  return nodes

def calculate_orientation(x1, y1, x2, y2):
    """Calcula a orientação (em radianos) entre dois pontos"""
    dx = x2 - x1
    dy = y2 - y1
    angle = math.atan2(dy, dx)  # Calcula o ângulo entre os dois pontos
    return angle

def quaternion_from_yaw(yaw):
    """Converte um ângulo (yaw) em um quaternion"""
    q = quaternion_from_euler(0, 0, yaw)
    return q

def send_msg(path,origin):
    path_msg = Path()
    path_msg.header.frame_id = "map"
    path_msg.header.stamp = rospy.Time.now()
    resolution = 1
    for i in range(len(path)):
        (x, y) = path[i]
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = rospy.Time.now()
        
        # Converter para coordenadas do mundo real
        pose.pose.position.x = y * resolution + origin.y #+ 0.5  ## tive que inverter aqui nao sei exatamente o pq, mas depois eu vejo
        pose.pose.position.y = x * resolution + origin.x #+ 0.5
        pose.pose.position.z = 0

        # Se não for o último waypoint, calcular a orientação para o próximo ponto
        if i < len(path) - 1:
            next_x, next_y = path[i + 1]
            yaw = calculate_orientation(y, x, next_y, next_x)
            quaternion = quaternion_from_yaw(yaw)
            pose.pose.orientation.x = quaternion[0]
            pose.pose.orientation.y = quaternion[1]
            pose.pose.orientation.z = quaternion[2]
            pose.pose.orientation.w = quaternion[3]
        else:
            # No último waypoint, manter a orientação anterior ou definir sem rotação
            pose.pose.orientation.w = 1.0  # Sem rotação

        path_msg.poses.append(pose)

    if path_msg.poses:
        rospy.loginfo_once("Publishing path with {0} waypoints...".format(len(path_msg.poses)))
        path_pub.publish(path_msg)
    else:
        rospy.logwarn("Path is empty. Nothing to publish.")

def visualize_grid_with_weights(grid_nodes, origin):
    #rospy.loginfo(f"tamanho do nodes {len(grid_nodes)}")
    for node in grid_nodes:
        marker = Marker()
        marker.header = Header(frame_id="map")
        marker.ns = "grid_weights"
        #rospy.loginfo(f"no : {grid_nodes[node[0],node[1]].peso}")
        #marker.id = hash(grid_nodes[node[0],node[1]].peso)  # Garante um ID único baseado na pose
        marker.id = random.randint(0,1000)  # Garante um ID único baseado na pose
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD

        # Texto com o peso do nó
        marker.text = str(grid_nodes[node[0],node[1]].peso)

        # Define a escala e a cor do texto
        marker.scale.z = 0.5  # Tamanho do texto
        marker.color.a = 1.0  # Opacidade do texto
        marker.color.r = 1.0  # Cor vermelha
        marker.color.g = 1.0  # Cor verde
        marker.color.b = 1.0  # Cor azul

        # Define a posição do texto
        marker.pose.position = Point(
            y=grid_nodes[node[0],node[1]].pose[0] + origin.x + 0.5,
            x=grid_nodes[node[0],node[1]].pose[1] + origin.y + 0.5,
            z=0.1  # Eleva ligeiramente o texto para evitar sobreposição com o grid
        )
        marker.pose.orientation = Quaternion(0, 0, 0, 1)

        # Publica o marcador no tópico de visualização
        grid_pub.publish(marker)

def apply_buffer_to_map(data):
    """Aplica um buffer ao redor de células ocupadas"""
    buffered_map = np.copy(data)
    height, width = data.shape

    for x in range(height):
        for y in range(width):
            if data[x, y] >= OCCUPIED_THRESHOLD:
                for i in range(-BUFFER_RADIUS, BUFFER_RADIUS + 1):
                    for j in range(-BUFFER_RADIUS, BUFFER_RADIUS + 1):
                        nx, ny = x + i, y + j
                        if 0 <= nx < height and 0 <= ny < width:
                            buffered_map[nx, ny] = OCCUPIED_THRESHOLD
    return buffered_map

def a_star_search(field, start, goal):
    """A* algorithm to find the shortest path from start to goal."""
    open_set = []
    heapq.heappush(open_set, (0, start))
    came_from = {}
    g_score = {start: 0}
    f_score = {start: heuristic(start, goal)}

    directions = [[1, 0], [0, 1], [-1, 0], [0, -1], [-1, -1], [-1, 1], [1, -1], [1, 1]]

    while open_set:
        _, current = heapq.heappop(open_set)

        if current == goal:
            return reconstruct_path(came_from, current)

        for direction in directions:
            neighbor = (current[0] + direction[0], current[1] + direction[1])

            if 0 <= neighbor[0] < field.shape[0] and 0 <= neighbor[1] < field.shape[1]:
                if field[neighbor[0], neighbor[1]] == 1:  # Skip obstacles
                    continue

                tentative_g_score = g_score[current] + 1
                if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g_score
                    f_score[neighbor] = tentative_g_score + heuristic(neighbor, goal)
                    heapq.heappush(open_set, (f_score[neighbor], neighbor))

    return []  # Return an empty path if no solution found


def heuristic(a, b):
    """Manhattan distance heuristic."""
    return abs(a[0] - b[0]) + abs(a[1] - b[1])

def reconstruct_path(came_from, current):
    """Reconstructs the path from start to goal."""
    path = [current]
    while current in came_from:
        current = came_from[current]
        path.append(current)
    path.reverse()
    return path


def get_worst_path_with_fallback(graph, nodes, start, field,goal,origin):
    """Wavefront planning with fallback to A*."""
    path = []
    actual = start
    path_node = None
    visited_nodes = []
    visited_nodes.append(actual)
    path.append(start)
    cont = 0
    backtrack = []
    free_nodes = [(x, y) for x, y in nodes if nodes[x, y].peso < float('inf') and [x, y] not in path]

    while free_nodes: # - test - 
        #rospy.loginfo(f"len visited + free : {len(visited_nodes)+len(free_nodes)} /  {len(nodes)} freenodes ->{len(free_nodes)} ")
        #if actual==goal:
        #    rospy.loginfo("Path found.")
        #    break
        best = float('-inf')
        stuck = True
        #rospy.logwarn("Actual_node -> {}".format(actual))
        free_neigh = [[x, y] for x, y in graph[actual[0],actual[1]] if [x, y] not in path]
        #rospy.logwarn("actual graph -> {}".format(free_neigh))

        for neighbor in graph[actual[0], actual[1]]:
            if nodes[neighbor[0], neighbor[1]].peso > best and neighbor not in path:
                best = nodes[neighbor[0], neighbor[1]].peso
                path_node = neighbor
                stuck = False

        if not stuck:  # Wavefront can continue
        #    rospy.logwarn("Wave encontrou o caminho -> {}".format(path_node))
            path.append(path_node)
            visited_nodes.append(actual)
            actual = path_node
          

        else:  # Wavefront is stuck, use A*
            #rospy.logwarn("Wavefront stuck. Falling back to A*.")
            # Find the nearest free node
            free_nodes = [(x, y) for x, y in nodes if nodes[x, y].peso < float('inf') and [x, y] not in path]
            
            for [x,y] in free_nodes:
               nodes[x,y].dist = heuristic(tuple([x,y]),tuple(actual))
            
            free_nodes.sort(key=lambda n: nodes[n[0], n[1]].dist)
            #rospy.logwarn(f"free_nodes {free_nodes} actual {actual}")
            
            if not free_nodes:
              break
            
            nearest_free = free_nodes[0] if free_nodes else None ## força a retorna o ultimo item da lista no caso é o mais loge possivel do goal.

            if nearest_free:
                fallback_path = a_star_search(field, tuple(actual), tuple(nearest_free))
                fallback_path = [list(node) for node in fallback_path] ## converte de um lista de tuples para uma lista de listas ...

                if fallback_path:
                    path.extend(fallback_path)
                    actual = fallback_path[-1]
                    visited_nodes.append(actual)
                    #rospy.logwarn("A* encontrou.")
                    #rospy.loginfo("caminho encontrado {}".format(fallback_path[-1]))
                    

                else:
                    rospy.logwarn("A* could not find a fallback path.")
                    break
        #if animated:    
        #  send_msg(path,Point(origin.x,origin.y,0))

    return path


def calculate_angle(p1, p2, p3):
    """
    Calcula o ângulo entre três pontos consecutivos.
    """
    v1 = (p2[0] - p1[0], p2[1] - p1[1])
    v2 = (p3[0] - p2[0], p3[1] - p2[1])
    dot_product = v1[0] * v2[0] + v1[1] * v2[1]
    mag_v1 = math.sqrt(v1[0] ** 2 + v1[1] ** 2)
    mag_v2 = math.sqrt(v2[0] ** 2 + v2[1] ** 2)
    if mag_v1 * mag_v2 == 0:
        return 0
    cos_theta = dot_product / (mag_v1 * mag_v2)
    return math.acos(max(-1, min(1, cos_theta)))

def calculate_path_score(start,goal,path):
    """
    Calcula a pontuação de um caminho com base em overlaps e mudanças de direção.
    
    path: lista de listas [[x1, y1], [x2, y2], ...]
    """
    overlaps = 0
    direction_changes = 0
    visited_points = set()

    for i in range(len(path) - 1):
        point = tuple(path[i])  # Converte o ponto para uma tupla para ser armazenado no set
        if point in visited_points:
            overlaps += 1
        visited_points.add(point)

        # Calcula mudanças de direção
        if i < len(path) - 2:
            angle = calculate_angle(path[i], path[i + 1], path[i + 2])
            if angle > math.pi / 4:  # Mudança de direção significativa (> 45 graus)
                direction_changes += 1

    return {
        "start":start,
        "goal":goal,
        "overlaps": overlaps,
        "direction_changes": direction_changes,
        "score": overlaps + direction_changes  # Você pode ajustar essa fórmula conforme necessário
    }

def save_score_to_file(data, filename="path_score.json"):
    rospack = rospkg.RosPack()
    dir = rospack.get_path('wavefront_ros')
    filename = dir+"/results/"+filename
    with open(filename, "w") as f:
        json.dump(data, f, indent=4)
        rospy.loginfo((f"Dados salvos em {filename}"))

def main(msg):

    width = msg.info.width
    height = msg.info.height
    resolution = msg.info.resolution
    origin = msg.info.origin.position
    field = np.array(msg.data).reshape((height, width))
    rospy.loginfo("Map loaded....")
    field = apply_buffer_to_map(field)
    field= np.where(field != 0, 1, 0) ## isso é so pra deixar como na minha implementacao
    rospy.loginfo("Building graph....")
    graph = construct_graph(field)
    #rospy.loginfo("Graph keys: {}".format(list(graph.keys())[:10]))  # Print first 10 keys

    rospy.loginfo("Graf complete....")
    rospy.loginfo("Starting nodes ....")
    all_nodes,nodes = init_nodes(field,init_peso=0)

    data = []

    for x in range(field.shape[0]):
        for y in range(field.shape[1]):
            #rospy.loginfo(f"field[{x},{y}] --> {field[x,y]}")
            if field[x,y] != 1:
                start = [x,y]
                for x_g in range(field.shape[0]):
                    for y_g in range(field.shape[1]):
                        if field[x_g,y_g] != 1:
                        
                            goal = [x_g,y_g]

                            rospy.loginfo("Start node : [{0},{1}] , goal : [{2},{3}]".format(start[0],start[1],goal[0],goal[1]))

                            rospy.loginfo("Starting Wavefront ....")

                            nodes = wavefront(nodes,goal,graph)

                            rospy.loginfo("Finding path ....")

                            path = get_worst_path_with_fallback(graph,nodes,start,field,goal,origin)

                            score = calculate_path_score(start,goal,path)

                            data.append(score)

                            rospy.loginfo("Path score : {} ".format(score))

    #starts = [[4,5]]
    #goals = [[4,10],[4,11]]

    #for start in starts:
    #    for goal in goals:
    #       
    #        rospy.loginfo("Start node : [{0},{1}] , goal : [{2},{3}]".format(start[0],start[1],goal[0],goal[1]))
    #        
    #        rospy.loginfo("Starting Wavefront ....")
    #        
    #        nodes = wavefront(nodes,goal,graph)
    #        
    #        rospy.loginfo("Finding path ....")
    #        
    #        path = get_worst_path_with_fallback(graph,nodes,start,field,goal,origin)
    #        
    #        score = calculate_path_score(start,goal,path)
    #        
    #        data.append(score)
    #        
    #        rospy.loginfo("Path score : {} ".format(score))

    df = pd.DataFrame(data)  
    rospack = rospkg.RosPack()
    dir = rospack.get_path('wavefront_ros')
    filename = dir+"/results/wavefront_a_start.csv"
    df.to_csv(filename, index=False) 
    rospy.loginfo((f"Dados salvos em {filename}"))   

    rate = rospy.Rate(1)
    
    while not rospy.is_shutdown():
      send_msg(path,msg.info.origin.position)
      rate.sleep()
    
    # print(field,path)

if __name__ == '__main__':
    #try:
    #  animated=bool(sys.argv[1])
    #except IndexError: 
    #  animated = False
    rospy.init_node("wavefront_cpp",anonymous=False)
    node_name = rospy.get_name()+"/"
    BUFFER_RADIUS_DEFAULT = 1
    BUFFER_RADIUS = int(rospy.get_param(node_name+"BUFFER_RADIUS", BUFFER_RADIUS_DEFAULT))
    animated = bool(rospy.get_param(node_name+"animated",False))

    path_pub = rospy.Publisher('/planned_path', Path, queue_size=10)
    rospy.Subscriber("/map", OccupancyGrid, main)
    
    rospy.spin()

