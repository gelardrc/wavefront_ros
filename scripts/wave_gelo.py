#!/usr/bin/env python
# -*- coding: utf-8 -*-
## codigo de teste para wavefront ##
import rospy
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped
import numpy as np
import matplotlib.pyplot as plt
import math
from tf.transformations import quaternion_from_euler

OCCUPIED_THRESHOLD = 50  # Valor mínimo para considerar uma célula ocupada
BUFFER_RADIUS = 1  # Células ao redor de obstáculos tratadas como ocupadas

class node:
  def __init__(self,pose,peso):
    self.pose = pose
    self.peso = peso
    self.parent = [] ## o parent agora e uma lista

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

def get_worst_path(graph,nodes,start):
  path = []
  actual = start
  path_node = None
  visited_nodes = []
  visited_nodes.append(actual)
  cont = 0
  backtrack = []
  while len(visited_nodes) < len(nodes):

    best = float('-inf')

    for vizinho in graph[actual[0],actual[1]]:


      if nodes[vizinho[0],vizinho[1]].peso > best and vizinho not in path:

         best = nodes[vizinho[0],vizinho[1]].peso

         path_node = vizinho
    
    ## ate aqui o funcionamento é o normal, ou seja ele desconsidera o fato de poder ficar preso

    if path_node != actual and path_node not in path: ## aqui eu fiz uma verificação estranha, eu meio que volto 

      if cont > 0:
        for i in backtrack:
          path.append(i)
        #path.append(actual)

      path.append(path_node)

      visited_nodes.append(actual)

      actual = path_node

      backtrack = []


      cont = 0

      #print('nao ha ninguem melhor',actual)

    else:
      #print('Vamos rezar')
      cont += 1
      if cont > len(path):
        break
      actual = path[-cont]
  
      backtrack.append(actual)



  return path

def printi(field,path):
    for x in range(field.shape[0]):
        for y in range(field.shape[1]):
          if field[x,y] == 1:
            plt.scatter(x,y,marker="s",color="black")
          else:
            field[x,y] = 0  
    x_i= []
    y_i =[]
    for i in path:
        x_i.append(i[0])
        y_i.append(i[1])
    
    plt.plot(x_i,y_i,color='r')

    plt.show()

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
        rospy.loginfo("Publishing path with {0} waypoints...".format(len(path_msg.poses)))
        path_pub.publish(path_msg)
    else:
        rospy.logwarn("Path is empty. Nothing to publish.")

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

def main(msg):

    width = msg.info.width
    height = msg.info.height
    resolution = msg.info.resolution
    origin = msg.info.origin.position
    field = np.array(msg.data).reshape((height, width))
    rospy.loginfo("Mapa carregado....")
    field = apply_buffer_to_map(field)
    field= np.where(field != 0, 1, 0) ## isso é so pra deixar como na minha implementacao
    rospy.loginfo("Contruindo grafo....")
    graph = construct_graph(field)
    rospy.loginfo("Grafo construido....")
    rospy.loginfo("Iniciando nodes ....")
    all_nodes,nodes = init_nodes(field,init_peso=0)

    goal = [33,33] # random goal 
    start = [33,38] # random start 

    rospy.loginfo("Start node : [{0},{1}] , goal : [{2},{3}]".format(start[0],start[1],goal[0],goal[1]))

    rospy.loginfo("Iniciando wavefront ....")

    nodes = wavefront(nodes,goal,graph)

    rospy.loginfo("Calculando rota ....")

    path = get_worst_path(graph,nodes,start)

    rospy.loginfo("Printing path ....")

    rate = rospy.Rate(1)
    
    while not rospy.is_shutdown():
      send_msg(path,msg.info.origin.position)
      rate.sleep()
    
    # print(field,path)

if __name__ == '__main__':
    rospy.init_node("wavefront_cpp",anonymous=False)
    path_pub = rospy.Publisher('/planned_path', Path, queue_size=10)
    rospy.Subscriber("/map", OccupancyGrid, main)
    
    rospy.spin()

