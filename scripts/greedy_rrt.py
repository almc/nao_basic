#!/usr/bin/env python

import numpy as np
import sys, random, math, pygame
from pygame.locals import *
from math import sqrt,cos,sin,atan2

# constants
DIM, DIM_X, DIM_Y = 2, 640, 480   # range 0 to 640, and 0 to 480
WINSIZE = [DIM_X, DIM_Y]          # could be any range for each var

# parameters
NUMNODES     = 2000
DELTA_IN     = np.array([0, 0])    # 15
DELTA_UT     = np.array([20, 20])  # 20
PEN_DIST_EXP = 5
PEN_DIST_OBS = 8
EXP_DIST     = 5
GOAL_DIST    = 10

# EPSILON_X = 20.0
# EPSILON_Y = 15.0
# DELTA_X   = 10.0
# DELTA_Y   = 7.0
# PEN_EXPANSION = 0.9
# PEN_INVALID   = 0.1
# PATCH_VALID   = np.array([])
# PATCH_INVALID = np.array([])

def dist(p1, p2):
    return np.linalg.norm(p1[0:2]-p2[0:2])

def raw_score(node1, node2):
    return 1.0/dist(node1, node2)

def generate_random():
    rand = np.array([np.random.uniform(DELTA_IN[0], DELTA_UT[0], 1)[0],
                     np.random.uniform(DELTA_IN[1], DELTA_UT[1], 1)[0], 0])
    sign = np.random.uniform(0, 1, 2)
    for r_i, r in enumerate(rand[:-1]):
        if sign[r_i] >= 0.5:
            rand[r_i] *= -1
    return rand

# def avg_score(nn, nodes):
#     box = pygame.Rect(nn[0], nn[1], int(DELTA_X), int(DELTA_Y))
#     delta_n_counter = 0
#     delta_s_counter = 0
#     for n_i, n in enumerate(nodes):
#         if box.collidepoint(n[0], n[1]): # check delta neighbourhood
#             delta_n_counter += 1
#             delta_s_counter += n[2]
#     print ">>>>>>>>>>>>>>FOUND THIS MANY NODES IN THE NEIGHBOURHOOD", delta_n_counter
#     if delta_n_counter >= 1:    # could be a parameter
#         avg_score = delta_s_counter / delta_n_counter
#         return 0.5*nn[2] + 0.5*avg_score # could be two parameters
#     else:
#         return nn[2]

def check_collision(node, obsta):
    for o in obsta:          # check node collision with every obstacle
        if (o.collidepoint(node[0], node[1])):
            return True         # return as soon as one of them is true
    return False                # if no obstacle collides return false

def check_unexplored(nn, nodes):
    for n_i, n in enumerate(nodes):
        d = dist(nn, n)
        if d < EXP_DIST:
            # print "explored node, rejected node, distance", n, nn, d
            return False
    return True

def check_goal(nn, goals):
    if dist(nn, goals) < GOAL_DIST: return True
    else: return False

def draw_obsta(screen, obsta, color):
    for o in obsta:
        pygame.draw.rect(screen, color, o, 0)

def draw_nodes(screen, nodes, color, node_radius):
    for n in nodes:
        pygame.draw.circle(screen, color, (int(n[0]),int(n[1])), node_radius, 2)

def draw_goals(screen, goals, color, node_radius):
    for g in goals:
        pygame.draw.circle(screen, color, (int(g[0]),int(g[1])), node_radius, 2)

def penalize_nodes(nn, nodes, pen_dist):
    for n_i, n in enumerate(nodes):
        d = dist(nn, n)
        if d < pen_dist:
            # print "penalizing node", n_i, nodes[n_i][2]
            nodes[n_i][2] *= d/pen_dist
            # print "score after penalization", nodes[n_i][2]
    return nodes

def organize_nodes(nodes):
    # nodes = np.sort(nodes, axis=0)[::-1]
    temp  = nodes[nodes[:,2].argsort()]
    nodes = temp[::-1]
    # print "nodes organized\n", nodes
    return nodes

def insert_node(nn, nodes):
    flag_inserted = False
    for p_i, p in enumerate(nodes):
        if nn[2] > p[2]: # if avg_score of new node is higher than p_i
            # print "adding node", nn
            nodes = np.insert(nodes, [p_i], nn, 0)
            flag_inserted = True
            break
    if flag_inserted == False:
        # print "score is worse than others"
        # print nodes
        # print nn
        nodes = np.append(nodes, [nn], 0)
    return nodes

def generate_newnode(nodes):
    rand = generate_random()
    nn = nodes[0] + rand    # new node, carefull with third coord still has prev raw_score

    # nodes = penalize_nodes(nn, nodes, PEN_DIST_EXP) # penalize nodes closer than PEN_DIST
    # nodes = organize_nodes(nodes)
    scaling_factor = 1.01
    while not check_unexplored(nn, nodes):
        # penalize node for being close to other nodes
        # print "penalizing node", nodes[0][2]

        nodes = penalize_nodes(nn, nodes, PEN_DIST_EXP) # penalize nodes closer than PEN_DIST
        nodes = organize_nodes(nodes)

        if (scaling_factor >= 2):
            scaling_factor = 2

        rand = scaling_factor*generate_random()
        nn = nodes[0] + rand
        scaling_factor *= scaling_factor

        # nodes[0][2] *= 0.5
        # nodes = organize_nodes(nodes)


        # print "score after penalization", nodes[0][2]
        # generate another node
        # rand = generate_random()
        # nn = nodes[0] + rand    # new node, carefull with third coord still has prev raw_score
        # print "newnode", nn
    return nn, nodes

## MAIN ##
def main():
    np.set_printoptions(precision=10, suppress=True)
    pygame.init()
    pygame.display.set_caption('RRT mod - Alejandro Marzinotto - June 2014')
    screen = pygame.display.set_mode(WINSIZE)
    white  = (255, 240, 200)
    black  = ( 20,  20,  40)
    red    = (192,   0,   0)
    green  = (  0, 192,   0)
    blue   = (  0,   0, 192)
    yellow = (192, 192,   0)
    node_radius = 3
    screen.fill(black)

    # variables
    goals = np.array([[DIM_X/2.0,DIM_Y*1.0/12.0, 1.0]]) # goal importance
    nodes = np.array([[DIM_X/2.0,DIM_Y*3.0/ 4.0, 0.0]]) # node raw_score
    nodes[0][2] = raw_score(nodes[0], goals[0])

    x, y = DIM_X*1.0/8.0, DIM_Y*1.0/8.0
    # obsta = [pygame.Rect(x, y, 380, 100), pygame.Rect(x, y, 100, 250)]
    # obsta = [pygame.Rect(x, y, 380, 100), pygame.Rect(x, y, 100, 250),
    #          pygame.Rect(x+400, y, 100, 250)]
    obsta = [pygame.Rect(x, y, 390, 100), pygame.Rect(x, y, 100, 250),
             pygame.Rect(x+400, y, 100, 250)]

    invalid = check_collision(nodes[0], obsta)
    assert invalid == False, "The initial pose is in a collision state"

    draw_obsta(screen, obsta, red)
    draw_nodes(screen, nodes, white, node_radius)
    draw_goals(screen, goals, green, node_radius)
    pygame.display.update()
    # a=raw_input()

    for i in range(NUMNODES):   # assumes that node[0] has the highest score
        # print ">>>>>>>>>>expansion number:", i, "node:", nodes[0]
        # a=raw_input()

        [nn, nodes] = generate_newnode(nodes)

        # raw_input()
        # revisar que el nodo generado no caiga dentro de patched area.
        # si cae dentro, penalizar todos los nodos involucrados, ordenar los nodos,
        # extraer el mejor y volver a expandir. (hasta que salgamos del problema)

        cn_i, cn = 0, nodes[0]  # closest node hypothesis
        for p_i, p in enumerate(nodes):
            if dist(p, nn) < dist(cn, nn):
                cn_i, cn = p_i, p
        # print "closest node found:", cn, cn_i


        # print "nodes before check_collision\n", nodes
        if check_collision(nn, obsta):
            # print ">>> in-valid node, penalizing"
            nodes = penalize_nodes(nn, nodes, PEN_DIST_OBS) # penalize nodes closer than PEN_DIST
            nodes = organize_nodes(nodes)
            pygame.draw.circle(screen, blue, (int(nn[0]),int(nn[1])), node_radius, 2)
            # print "nodes after check_collision\n", nodes
            # a=raw_input()
        else:
            # print ">>> valid node, scoring"
            # print goals[0]
            # print nn
            # print "***************"
            nn[2] = raw_score(nn, goals[0]) # overwriting raw_score of node who generated it
            # nn[2] = avg_score(nn, nodes)    # overwriting raw_score of the goal heuristic
            pygame.draw.line(screen, white, nodes[0][0:2], nn[0:2])
            nodes = insert_node(nn, nodes)
            # print "new node list:", nodes
            pygame.draw.circle(screen, yellow, (int(nn[0]),int(nn[1])), node_radius, 2)
        # print "nodes after check_collision\n", nodes
        pygame.display.update()
        if check_goal(nn, goals[0]):
            print "found path, finishing"
            break




    # for i in range(NUMNODES):
    #     rand = random.random()*640.0, random.random()*480.0
    #     nn = nodes[0]
    #     for p in nodes:
    #        if dist(p,rand) < dist(nn,rand):
    #           nn = p
    #     newnode = step_from_to(nn,rand)
    #     nodes.append(newnode)
    #     pygame.draw.line(screen,white,nn,newnode)
    #     pygame.display.update()
    #     #print i, "    ", nodes

    #     for e in pygame.event.get():
    #        if e.type == QUIT or (e.type == KEYUP and e.key == K_ESCAPE):
    #           sys.exit("Leaving because you requested it.")


# if python says run, then we should run
if __name__ == '__main__':
    main()
