#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# {Erik Molitor}
# {961104-3879}
# {emolitor@kth.se}
from dubins import Car, step
from math import pi, sqrt
from random import randint, uniform

class Node:
    def __init__(self, x, y, theta=None, time=None, phi = None, parent=None):
      self.x = x 
      self.y = y
      self.steering = phi
      self.time = time
      self.heading = theta
      self.parent = parent

def randcoord():
    if randint(0,100) > goal_priority: # priority to choose goal coordinate
        x = uniform(xlb, xub) 
        y = uniform(ylb, yub)
        return x, y
    else: 
        return xt, yt #goal coordinates
    

def distance(node1, node2): # callculate distance between 2 nodes 
    dist = sqrt((node1.x-node2.x)**2+(node1.y-node2.y)**2)
    return dist


def closestNode(node):
    min_dist = 100
    for idx, item in enumerate(nodelist):
        d = distance(item, node)# calcualate distance between two nodes 
        if d < min_dist: # check if distance is less than the last closest node so far 
            min_dist = d # if closer, set new minimum distance
            index = idx # remember index of closest node
    return nodelist[index]


def goToNode(car, bNode,aNode):
    c_node = None
    min_dist = 100 #start value 
    for phi in [-pi/4, 0, pi/4]:
        x = bNode.x
        y = bNode.y
        theta = bNode.heading
        crash = False     
        for deltaL in range(stepInIter): # step in each turn
            xn, yn, thetan = step(car, x, y, theta, phi, dt)
            x, y, theta = xn, yn, thetan 
            if collisionCheck(car, x, y) == True: #check if step cause collision
                crash = True # if collsion break to new steering value
                break

        test_node = Node(x, y)
        #se whitch path get closest to aNode
        dist = distance(aNode, test_node)
        if dist < min_dist and crash == False:
            min_dist = dist
            c_node = Node(x,y, theta,(bNode.time +stepInIter*dt) , phi, bNode)
         
    # if no path exist remove bNode from nodelist 
    if c_node == None: 
        nodelist.remove(bNode)
        return
    return c_node


def collisionCheck(car, x, y):
    crash = False
    #se if path go into obstacle block
    for block in car.obs:
        if block[2]**2+0.5 >= (x-block[0])**2+(y-block[1])**2: # 0.5 extra security distance from obstacle
            crash = True
            continue      
    # See if path runs of end of map
    #x axis
    if x <= xlb or x >= xub:
        crash = True
    #y axis
    if y <= ylb or y >= yub:
        crash = True 
    return crash

def path(current, controls, times):
    while current is not None: #iterate back from node who got to goal thru parents to start
        controls.append(current.steering) #extract steering 
        times.append(current.time) # extract time 
        current = current.parent #set next iteration node
    controls = controls[::-1] #reverse order because we have goal to start right now
    times = times[::-1] # same for time 
    return controls, times

def solution(car):
    ''' <<< write your code below >>> '''
    global xlb, xub, ylb, yub, xt, yt, x0, y0, nodelist, stepInIter, dt, goal_priority
    
    
    # initial state   
    controls=[]
    times=[]
    xlb, xub, ylb,yub = car.xlb,car.xub,car.ylb,car.yub # boundary values of map
    xlb, xub, ylb, yub = xlb-1, xub+1, ylb+1, yub-1 # security distance from border of map
    x0, y0 = car.x0, car.y0 # start coordinate
    xt,yt =car.xt, car.yt  # goal coordinate  
    stepInIter = 50 # precision in curve taking (steps in each curve)
    dt = 0.01   #time step
    goal_priority = 5 # procent of guesses is the end point
    theta_start = 0 #start value for theta
    phi_start = 0 # start valuse for phi 
    iterations = 50000 #how many nodes should be generated to find solution
    start_node = Node(x0,y0, theta_start, 0) #start node
    end_node = Node(xt, yt) # end node, only coordinates needed to solve problem
    nodelist = [start_node] # list of all nodes 

    # generate node mesh 
    goal = False
    for i in range(iterations) or goal == True:
        #random new coordinate node
        new_x,new_y = randcoord()
        aNode = Node(new_x,new_y)
        #closest node to aNode
        bNode = closestNode(aNode)
        # try to drive to new node but result in a lite ofset called cNode
        c_node = goToNode(car, bNode, aNode)
        #check if cNode exist for this (if no crash), if exist append cNode to nodelist
        if c_node:
            nodelist.append(c_node)
            if distance(end_node, c_node) <= 1.25:  #check if goal is within reach 
                goal = True #break for-loop 
                break 
        # do next random node 


    #extract controls and times from final correct path 
    controls, times = path(c_node, controls, times)
    controls = controls[1:] # remove dubbled start value
    print(times[-1])
    ''' <<< write your code above >>> '''

    return controls, times
