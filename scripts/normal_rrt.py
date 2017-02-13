#!/usr/bin/env python

import numpy as np
import sys, random, math, pygame
from pygame.locals import *
from math import sqrt,cos,sin,atan2

# constants
DIM, DIM_X, DIM_Y = 2, 640, 480   # range 0 to 640, and 0 to 480
WINSIZE = [DIM_X, DIM_Y]          # could be any range for each var

# parameters
NUMNODES     = 5000
DELTA_IN     = np.array([0, 0])    # 15
DELTA_UT     = np.array([20, 20])  # 20
PEN_DIST_EXP = 5
PEN_DIST_OBS = 8
EXP_DIST     = 5
GOAL_DIST    = 10
EPSILON      = 14.0

def dist(p1, p2):
    return np.linalg.norm(p1[0:2]-p2[0:2])

def step_from_to(p1,p2):
    if dist(p1,p2) < EPSILON:
        return np.array([p2[0], p2[1], 0.0])
    else:
        theta = atan2(p2[1]-p1[1],p2[0]-p1[0])
        return np.array([p1[0] + EPSILON*cos(theta), p1[1] + EPSILON*sin(theta),0.0])

def generate_random():
    rand = np.array([np.random.uniform(DELTA_IN[0], DELTA_UT[0], 1)[0],
                     np.random.uniform(DELTA_IN[1], DELTA_UT[1], 1)[0], 0])
    sign = np.random.uniform(0, 1, 2)
    for r_i, r in enumerate(rand[:-1]):
        if sign[r_i] >= 0.5:
            rand[r_i] *= -1
    return rand

def check_collision(node, obsta):
    for o in obsta:          # check node collision with every obstacle
        if (o.collidepoint(node[0], node[1])):
            return True         # return as soon as one of them is true
    return False                # if no obstacle collides return false

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


def main():
    np.set_printoptions(precision=10, suppress=True)
    pygame.init()
    pygame.display.set_caption('RRT regular - Alejandro Marzinotto, La Valle- February 2017')
    screen = pygame.display.set_mode(WINSIZE)
    white  = (255, 240, 200)
    black  = ( 20,  20,  40)
    red    = (192,   0,   0)
    green  = (  0, 192,   0)
    blue   = (  0,   0, 192)
    yellow = (192, 192,   0)
    node_radius = 3
    screen.fill(black)
    a=raw_input()
    # variables
    goals = np.array([[DIM_X/2.0,DIM_Y*1.0/12.0, 1.0]]) # goal importance
    nodes = np.array([[DIM_X/2.0,DIM_Y*3.0/ 4.0, 0.0]]) # node raw_score

    x, y = DIM_X*1.0/8.0, DIM_Y*1.0/8.0
    obsta = [pygame.Rect(x, y, 390, 100), pygame.Rect(x, y, 100, 250),
             pygame.Rect(x+400, y, 100, 250)]
    invalid = check_collision(nodes[0], obsta)
    assert invalid == False, "The initial pose is in a collision state"

    draw_obsta(screen, obsta, red)
    draw_nodes(screen, nodes, white, node_radius)
    draw_goals(screen, goals, green, node_radius)
    pygame.display.update()


    for i in range(NUMNODES):
        print "iteration", i
        rand = random.random()*640.0, random.random()*480.0 # DIM_X, DIM_Y
        nn = nodes[0]
        for p in nodes:
            if dist(p,rand) < dist(nn,rand):
                nn = p
        newnode = step_from_to(nn,rand)
        if check_collision(newnode, obsta):
            pygame.draw.circle(screen, blue, (int(newnode[0]),int(newnode[1])), node_radius, 2)
        else:
            pygame.draw.line(screen, white, nn[0:2], newnode[0:2])
            nodes = np.append(nodes, [newnode], 0)
            pygame.draw.circle(screen, yellow, (int(newnode[0]),int(newnode[1])), node_radius, 2)

        pygame.display.update()
        if check_goal(newnode, goals[0]):
            print "found path, finishing"
            break

        for e in pygame.event.get():
            if e.type == QUIT or (e.type == KEYUP and e.key == K_ESCAPE):
                sys.exit("Leaving because you requested it.")

if __name__ == '__main__':
    main()
