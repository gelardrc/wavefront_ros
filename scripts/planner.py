#!/usr/bin/env python3
# -*- coding: utf-8 -*-
## codigo de teste para wavefront ##
import rospy
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped,Point,Quaternion
from visualization_msgs.msg import Marker
from std_msgs.msg import Header
import numpy as np
import random
import time
import matplotlib.pyplot as plt
import math
import json
import rospkg
import heapq
from tf.transformations import quaternion_from_euler
import sys

OCCUPIED_THRESHOLD = 50  # Valor mínimo para considerar uma célula ocupada
#BUFFER_RADIUS = 1 # Células ao redor de obstáculos tratadas como ocupadas

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
  rospy.loginfo(f"wavefront nodes {len(nodes)}")
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
        pose.pose.position.x = y * resolution + origin.x #+ 0.5  ## tive que inverter aqui nao sei exatamente o pq, mas depois eu vejo
        pose.pose.position.y = x * resolution + origin.y #+ 0.5
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

def send_start_goal(start, goal, origin):
    # Criando dois marcadores independentes
    start_marker = Marker()
    goal_marker = Marker()

    # Configurações gerais para ambos os marcadores
    for marker in [start_marker, goal_marker]:
        marker.header = Header(frame_id="map")
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.scale.x = 1.0
        marker.scale.y = 1.0
        marker.scale.z = 1.0
        marker.color.a = 1.0  # Marcador opaco
        marker.pose.orientation = Quaternion(x=0, y=0.7071, z=0, w=0.7071)

    # Configurações específicas do marcador de início
    start_marker.id = 1
    start_marker.color.r = 0.0
    start_marker.color.g = 0.0
    start_marker.color.b = 1.0
    start_marker.pose.position = Point(
        x=start[1] + origin.x + 0.5, # it need to be inverted because of how gazebo works
        y=start[0] + origin.y + 0.5, 
        z=0
    )

    # Configurações específicas do marcador de objetivo
    goal_marker.id = 2
    goal_marker.color.r = 1.0
    goal_marker.color.g = 0.0
    goal_marker.color.b = 0.0
    goal_marker.pose.position = Point(
        x=goal[1] + origin.x + 0.5, 
        y=goal[0] + origin.y + 0.5, 
        z=0
    )

    # Publicando os marcadores
    s_g_pub.publish(start_marker)
    s_g_pub.publish(goal_marker)

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

def apply_buffer_to_map(data,buffer):
    """Aplica um buffer ao redor de células ocupadas"""
    buffered_map = np.copy(data)
    height, width = data.shape

    for x in range(height):
        for y in range(width):
            if data[x, y] >= OCCUPIED_THRESHOLD:
                for i in range(-buffer, buffer + 1):
                    for j in range(-buffer, buffer + 1):
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

def reconstruct_path_ba(came_from_fwd, meeting, came_from_bwd):
    """Reconstructs the path from start to goal."""
    path_fwd = []
    path_bwd = []
    current = meeting

    while current is not None:
        path_fwd.append(current)
        current = came_from_fwd.get(current)

    current = came_from_bwd[meeting]

    while current is not None:
        path_bwd.append(current)
        current = came_from_bwd.get(current)

    path_fwd.reverse()
    return path_fwd + path_bwd

def bidirectional_a_star(field, start,  goal): #f=g+h, g = custo percorrio até o nó atual; h = custo do nó atual até o objetivo
    """Implementação do algoritmo"""
    open_list_fwd = [] #criar a fila de prioridade
    open_list_bwd = []
    heapq.heappush(open_list_fwd, (0, start)) #adicionar o ponto inicial a lista de prioridade com seu f_score
    heapq.heappush(open_list_bwd, (0, goal)) #adiciona o ponto final a lista de prioridade como se fosse o inicial

    came_from_fwd = {} #saber de qual nó o atual veio
    came_from_bwd = {}

    g_score_fwd = {start: 0}
    g_score_bwd = {goal: 0}

    directions = [[1, 0], [0, 1], [-1, 0], [0, -1], [-1, -1], [-1, 1], [1, -1], [1, 1]] #posição dos vizinhos ao redor do nó atual

    while open_list_fwd and open_list_bwd:

        if open_list_fwd:
            _, current_fwd = heapq.heappop(open_list_fwd)

            if current_fwd in came_from_bwd:
                return reconstruct_path_ba(came_from_fwd, current_fwd, came_from_bwd)

            for direction in directions: 
                neighbor = (current_fwd[0] + direction[0], current_fwd[1] + direction[1])

                if 0 <= neighbor[0] < field.shape[0] and 0 <= neighbor[1] < field.shape[1]: #verifica se o vizinho stá na grid
                    if field[neighbor[0], neighbor[1]] == 1: 
                        continue #se for um obstáculo ele ignora
                    
                    new_cost = g_score_fwd[current_fwd] + 1 #o novo custo é igual ao custo até agora + 1

                    if neighbor not in g_score_fwd or new_cost < g_score_fwd[neighbor]:
                        came_from_fwd[neighbor] = current_fwd
                        g_score_fwd[neighbor] = new_cost 
                        f_score_fwd = g_score_fwd[neighbor] + heuristic(neighbor, goal)
                        heapq.heappush(open_list_fwd, (f_score_fwd, neighbor)) 

        if open_list_bwd:
            _, current_bwd = heapq.heappop(open_list_bwd)

            if current_bwd in came_from_fwd:
                return reconstruct_path_ba(came_from_fwd, current_bwd, came_from_bwd)

            for direction in directions: 
                neighbor = (current_bwd[0] + direction[0], current_bwd[1] + direction[1])

                if 0 <= neighbor[0] < field.shape[0] and 0 <= neighbor[1] < field.shape[1]: #verifica se o vizinho stá na grid
                    if field[neighbor[0], neighbor[1]] == 1: 
                        continue #se for um obstáculo ele ignora
                    
                    new_cost = g_score_bwd[current_bwd] + 1 #o novo custo é igual ao custo até agora + 1

                    if neighbor not in g_score_bwd or new_cost < g_score_bwd[neighbor]:
                        came_from_bwd[neighbor] = current_bwd
                        g_score_bwd[neighbor] = new_cost 
                        f_score_bwd = g_score_bwd[neighbor] + heuristic(neighbor, start)
                        heapq.heappush(open_list_bwd, (f_score_bwd, neighbor))   
    return []

def wavefront_search(graph,nodes,start,origin,goal,field,method,stop_criteria):
   visited_nodes = []
   free_nodes = [] # init free_nodes
   path = []
   backtrack_c = 0 # init backtrack counter
   backtrack = []  
   neigh = None
   visited_nodes.append(start) ## insert start in visitedo nodes
   actual = start # the first node to be verify 
   free_nodes = [[x,y] for [x,y] in nodes if [x,y] not in visited_nodes]
   start_time = time.time()
   while free_nodes and (time.time()-start_time) < stop_criteria: # while free_nodes exists continue - >  try to cover all area 
    
    neighbors = graph[actual[0],actual[1]] ## returns all possible actaul's neighbors from graph

    valid_neighbors = [neighbor for neighbor in neighbors if neighbor not in visited_nodes] ## check if neighbor are not in visited nodes 

    if valid_neighbors: ## if there is a valid neighbor algorithm will conitnue if not we try to call backtracking or another algotihm.
      
      if backtrack: ## if there something in backtracking we need to add it to path
        path.extend(backtrack) ## add backtrack list to path list
        backtrack=[]   ## init backtrack again
        backtrack_c = 0 ## reset backtrack counter

      best_neighbor = max(valid_neighbors, key=lambda n: nodes[n[0], n[1]].peso) ## return the "best" neighbor based on max peso ()
      
      visited_nodes.append(best_neighbor) ## add node in visited_nodes

      path.append(best_neighbor)  # Add to path

      actual = best_neighbor ## update actual to next loop

      free_nodes.remove(actual) # update free_nodes


    else: ## there is no free valid nodes in neighbors so we need to back track to find a valid one 
        #rospy.logwarn(f"Stuck at {actual}. Attempting backtracking.")
        if path:
          
          if method == "classic":
            backtrack_c += 1 # move one step back from actual node 
            #rospy.loginfo(f"backtrack count : {backtrack_c}")
            actual = path[-backtrack_c]  # Backtrack to the previous node
            #rospy.loginfo(f"actual node: {path[-backtrack_c]}")
            backtrack.append(actual)
          
          else: ## IMPROVE IT IN FUTURE !
            if backtrack_c > 20:
                #rospy.loginfo_once("Backtracking overflow, path will now be choose by :{method}")

                next_node = get_valid_node(nodes=nodes,free_nodes=free_nodes,actual=path[-1])

                if method == "a*":
                  backtrack = a_star_search(field, tuple(path[-1]), tuple(next_node))
                if method == "ba*":
                  backtrack = bidirectional_a_star(field, tuple(path[-1]), tuple(next_node))

                actual = next_node

            else:
              backtrack_c += 1 # move one step back from actual node 
              #rospy.loginfo(f"backtrack count : {backtrack_c}")
              actual = path[-backtrack_c]  # Backtrack to the previous node
              #rospy.loginfo(f"actual node: {path[-backtrack_c]}")
              backtrack.append(actual)
        else:
          rospy.logerr("No path to continue. Terminating.")
          break
      
    if animated:    
      send_msg(path,Point(origin.x,origin.y,0))
      send_start_goal(start,goal,origin)
      #time.sleep(0.5)
      #visualize_grid_with_weights(nodes,origin) 


   
   return path

def get_valid_node(nodes, free_nodes, actual):
    # Get the maximum peso in free_nodes (nodes with the highest weight)
    #max_weight = max(nodes[x, y].peso for x, y in free_nodes)  # This returns the max peso in free_nodes
    
    # Filter out free_nodes to include only nodes with the max peso
    #free_nodes = [(x, y) for x, y in free_nodes if nodes[x, y].peso == max_weight]
    
    # Update the distance for each node in free_nodes using the heuristic
    for x, y in free_nodes:
        nodes[x, y].dist = heuristic((x, y), tuple(actual))  # Calculate distance from current node
    
    # Sort free_nodes by the distance (dist)
    free_nodes.sort(key=lambda n: nodes[n[0], n[1]].dist)
    
    # Check if free_nodes is not empty and return the nearest node
    if free_nodes:
        nearest_free = free_nodes[0]  # The first item is the one with the smallest distance
        return nearest_free
    else:
        rospy.logwarn("No valid free nodes found.")
        return None

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

def random_walk(start,goal,nodes,graph,origin,field,stop_criteria):
  path = [] # init path 
  
  visited_nodes = [] # init visited_nodes
  
  actual = start # set actual to start
  
  visited_nodes.append(start)  # append it to visited_nodes

  path.append(start) # append it to path
  
  backtrack = []
  backtrack_c = 0
  
  free_nodes = [[x,y] for [x,y] in nodes if [x,y] not in visited_nodes]
  start_time = time.time()
  while free_nodes and (time.time()-start_time) < stop_criteria:
    
    valid_nodes = [n for n in graph[actual[0],actual[1]] if n not in visited_nodes and n in free_nodes]
    
    if valid_nodes: # verify if there are valid nodes 
      #rospy.loginfo(f"valid_nodes : {valid_nodes}")
      if backtrack: ## if there something in backtracking we need to add it to path
        for point in backtrack: ## eliminates freenodes already visited by backtrack
          if point in free_nodes:
            free_nodes.remove(point)

        path.extend(backtrack) ## add backtrack list to path list
        backtrack=[]   ## init backtrack again
        backtrack_c = 0 ## reset backtrack counter
        

      actual = random.choice(valid_nodes) # actual is random set to any valid_node
      
      #rospy.loginfo(f"actual {actual}")

      #rospy.loginfo(f"free_nodes {free_nodes}")

      path.append(actual) ## add path

      free_nodes.remove(actual) # update free_nodes

    else: ## there is no free valid nodes in neighbors so we need to back track to find a valid one 
        valid_nodes = [n for n in graph[actual[0],actual[1]]]
        #rospy.loginfo(f"actual dir {actual}")
        #rospy.loginfo(f"valid_nodes dir {valid_nodes}")
        if valid_nodes:
          actual = random.choice(valid_nodes)
          path.append(actual)

        if actual in free_nodes:
          free_nodes.remove(actual)

    if animated:    
      send_msg(path,Point(origin.x,origin.y,0))
      send_start_goal(start,goal,origin)
      #time.sleep(0.5)
      #visualize_grid_with_weights(nodes,origin) 
      


  return path

def calculate_path_score(start,goal,path,time,nodes):
    """
    Calcula a pontuação de um caminho com base em overlaps e mudanças de direção.
    
    path: lista de listas [[x1, y1], [x2, y2], ...]
    """
    overlaps = 0
    direction_changes = 0
    visited_points = set()
    
    not_visited = [[x,y] for [x,y] in nodes if [x,y] not in path]
   
    coverage_area = 100 - (100*(len(not_visited)-1))/(len(nodes)) # del start
    
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
        "time":time,
        "coverage_area": coverage_area,
        "score": overlaps + direction_changes  # Você pode ajustar essa fórmula conforme necessário
        
    }

def save_score_to_file(data, filename,start,goal,method):
    rospack = rospkg.RosPack()
    dir = rospack.get_path('wavefront_ros')
    filename= method +"_"+ str(start) + "_" + str(goal) +"_"+  filename
    filename = dir+"/results/"+filename
    with open(filename, "w") as f:
        json.dump(data, f, indent=4)
        rospy.loginfo((f"Dados salvos em {filename}"))

def main(msg):


    rospy.loginfo(f"\033[93mWavefront Method: {method}\033[0m")
    
    width = msg.info.width
    height = msg.info.height
    resolution = msg.info.resolution
    origin = msg.info.origin.position
    field = np.array(msg.data).reshape((height, width))
    rospy.loginfo("Map loaded....")
    field = apply_buffer_to_map(field,BUFFER_RADIUS)
    field= np.where(field != 0, 1, 0) ## isso é so pra deixar como na minha implementacao
    rospy.loginfo("Building graph....")
    graph = construct_graph(field)
    #rospy.loginfo("Graph keys: {}".format(list(graph.keys())[:10]))  # Print first 10 keys

    rospy.loginfo("Graf complete....")
    rospy.loginfo("Starting nodes ....")
    all_nodes,nodes = init_nodes(field,init_peso=0)



    default_goal  = [29,44] # random goal 
    default_start = [34,25] # random start 

    start = eval(rospy.get_param("start", default_start))

    if field[start[0],start[1]] == 1:
      rospy.loginfo(f"Bad start, {start} it's not in free_space")
      while field[start[0],start[1]] == 1:
        rospy.loginfo_once("Searching a valid one..")
        start = [random.randint(0,height-1),random.randint(0,width-1)]
      rospy.loginfo("Changing start for --> {}".format(start))

    
    goal =  eval(rospy.get_param("goal",   default_goal))


    if field[goal[0],goal[1]] == 1:
      rospy.loginfo(f"Bad goal, {goal} it's not in free_space")
      while field[goal[0],goal[1]] == 1:
        rospy.loginfo_once("Searching a valid one...")
        goal = [random.randint(0,height-1),random.randint(0,width-1)]
      rospy.loginfo("Changing goal for --> {}".format(goal))

    rospy.loginfo("Start node : [{0},{1}] , goal : [{2},{3}]".format(start[0],start[1],goal[0],goal[1]))

    rospy.loginfo("Starting Wavefront ....")

    begin = time.time()

    nodes = wavefront(nodes,goal,graph)

    stop_criteria = 60 # one minute 

    rospy.loginfo("Finding path ....")

    if method == "random_walk":
      path = random_walk(start,goal,nodes,graph,origin,field,stop_criteria)
    else:
      path = wavefront_search(graph,nodes,start,origin,goal,field,method,stop_criteria)
    
    

    end = time.time() - begin
    rospy.loginfo("Calculating score ....")
    score = calculate_path_score(start,goal,path,end,nodes)
    save_score_to_file(score, "score.json",start,goal,method)
    rospy.loginfo("Path score : {} ".format(score))

    rospy.loginfo("Printing path ....")

    rate = rospy.Rate(1)
    
    while not rospy.is_shutdown():
      send_msg(path,msg.info.origin.position)
      send_start_goal(start,goal,origin)
      #send_start_goal([1,1],goal,origin)
      visualize_grid_with_weights(nodes,origin)
      #rospy.loginfo(f"field[1,1] - {field[1,1]}")
      #rospy.loginfo(f"field[0,0] - {field[0,0]}")

      
      rate.sleep()
    
    # print(field,path)


if __name__ == '__main__':
    
    rospy.init_node("wavefront_cpp",anonymous=False)
    node_name = rospy.get_name()+"/"


    BUFFER_RADIUS_DEFAULT = 1
    BUFFER_RADIUS = int(rospy.get_param(node_name+"BUFFER_RADIUS", BUFFER_RADIUS_DEFAULT))
    animated = bool(rospy.get_param(node_name+"animated",False))
    method = rospy.get_param("method","classic") ## you can set paramter to classic, a* or ba*, "random_walk" .


    path_pub = rospy.Publisher('/planned_path', Path, queue_size=10)
    s_g_pub = rospy.Publisher('/s_g', Marker, queue_size=10)
    grid_pub = rospy.Publisher("/weigh_grid", Marker)

    rospy.Subscriber("/map", OccupancyGrid, main)

    
    rospy.spin()

