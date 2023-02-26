#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu Oct 27 21:47:22 2022
@author: matthewbungate
"""
from gurobipy import *
import math
import random
from random import randint
from random import seed
import numpy as np
import matplotlib
import matplotlib.pyplot as plt
from matplotlib.animation import PillowWriter
from matplotlib.animation import FuncAnimation

import time


random.seed(1)
EPS = 0.0001
# How many cuts do you want to make before showing the master problem
show_frequency = 0
# Data to be modified
numberCustomers = 20
numberTrucks = 1 #Model for multiple trucks is not working.
numberDrones = 3
# Speed ratio: how much quicker is the drone than the truck
SR = 1
# Drone limit: furthest distnace a drone can travel before returning
drone_limit = 60
def shortest_route(numberCustomers, numberTrucks, numberDrones, SR, drone_limit):
    drone_limit = float(drone_limit)
    # Sets
    C = range(numberCustomers)
    T = range(numberTrucks)
    D = range(numberDrones)
    def findFactors(x):
        factors = []
        for i in range(x):
            i += 1
            if x % i == 0:
                factors.append(i)
        return factors
    # Generates the an array where dist[i][j] is the distance between i and j
    def distGen(customers, n, dist):
        for i in range(n):
            dist.append([])
            iNode = customers[i]

            for j in range(n):
                if i == j:
                    dist[i].append(200)
                else:
                    xDist = customers[i][0] - customers[j][0]
                    yDist = customers[i][1] - customers[j][1]
                    dist[i].append(int(math.hypot(xDist, yDist)))
        return dist
    # Creates an instance of customers and distance
    # numberCustomers - number of customers in the isntance
    # numberTrucks - the number of trucksin the instance
    # random - Determines whether the instance is random or not. 'r' if random, 'u' if uniform
    def instanceGeneration(numberCustomers, numberTrucks, random):
        customers = []
        dist = []
        N = range(numberCustomers)
        T = range(numberTrucks)
        rowsCols = [-1, -1]
        factors = findFactors(numberCustomers)
        size = len(factors)
        if size % 2 == 0:
            rowsCols[0] = factors[(size//2)-1]
            rowsCols[1] = factors[size//2]
        else:
            rowsCols[0] = factors[(size//2)+1]
            rowsCols[1] = factors[(size//2)+1]
        if random == 'u':
        # Customer node generation
            for i in range(rowsCols[1]):
                xVal = (100*i)//(rowsCols[1]-1)
            for j in range(rowsCols[0]):
                yVal = (100*j)//(rowsCols[0]-1)
            index = i * rowsCols[1] + j
            customers.append((xVal, yVal))
            dist = distGen(customers, numberCustomers, [])
            # Distance generation
            for i in range(numberCustomers):
                dist.append([])
                iNode = customers[i]
                for j in range(numberCustomers):
                    jNode = customers[j]
                    xDist = iNode[0] - jNode[0]
                    yDist = iNode[1] - jNode[1]
                    distij = math.hypot(xDist, yDist)
                    dist[i].append(int(distij))
            minDist = []
            for i in C:
                minDist.append([])
                for j in C:
                    minDist[i].append(max(min(dist[i][c] + dist[c][j] - dist[i][j] for c in C), 0))
            return (customers, dist, minDist)
        else:
        # Randomly generated customers
        # Set the node to the centre of the problem
            customers.append([50,50])
            for c in range(numberCustomers - 1):
                xVal = randint(0, 100)
                yVal = randint(0, 100)
                node = (xVal, yVal)
                customers.append(node)
            dist = distGen(customers, numberCustomers, [])
            minDist = [] 
            for i in C:
                minDist.append([])
                for j in C:
                    minDist[i].append(max(min(dist[i][c] + dist[c][j] - dist[i][j] for c in C), 0))
            return (customers, dist, minDist)


    def valid_drone_routes(dist, drone_limit):
        valid_drone_nodes = []
        for i in C:
            for j in C:
                if i != j:
                    if dist[i][j] > drone_limit:
                        continue
                    else:
                        for k in C:
                            if j != k:
                                if dist[i][j] + dist[j][k] < drone_limit:
                                    valid_drone_nodes.append([i,j,k])
        return valid_drone_nodes



    def tour_nodes(start, active_arcs, nodes):
        for i in active_arcs:
            if start == i[0]:
                if i[1] in nodes:
                    return nodes
                else:
                    nodes.append(i[1])
                    return tour_nodes(i[1], active_arcs, nodes)

    # Start timer
    start = time.time()

    # Create the instance
    instance = instanceGeneration(numberCustomers, numberTrucks, 'r')
    customers = instance[0]
    dist = instance[1]
    minDist = instance[2]

    valid_drone_nodes = valid_drone_routes(dist, drone_limit)

    minNodes = math.ceil(numberCustomers*(numberTrucks/(numberTrucks+numberDrones)))

    removal_nodes = []
    for i in valid_drone_nodes:
        if i[1] not in removal_nodes:
            removal_nodes.append(i[1])

    required_nodes = []
    for i in range(numberCustomers):
        if i not in removal_nodes and i != 0:
            required_nodes.append(i)

    # Create the inital plot with the customers
    xVals = []
    yVals = []
    for i in customers:
        xVals.append(i[0])
        yVals.append(i[1])
    plt.scatter(xVals[0], yVals[0], c='r', marker='s')
    plt.scatter(xVals[1:], yVals[1:], c='b')
    plt.xlim([0,100])
    plt.ylim([0,100])
    #for i in C:
    #    plt.annotate(i, (customers[i][0]+1, customers[i][1]+1))
    plt.show()

    # Callback function called by the master problem to eliminate subtours
    def Callback(model, where):
        if where == GRB.Callback.MIPSOL:
            XV = model.cbGetSolution(X)
            for t in T:
                active_arcs = []
                for i in C:
                    for j in C:
                        if XV[i,j,t] > 0.9:
                            active_arcs.append([i,j])
                for c in range(numberCustomers):
                    for a in active_arcs:
                        if c == a[0]:
                            route = tour_nodes(c, active_arcs, [c])
                            if 0 not in route:
                                arcs_used = []
                                for r in range(len(route) - 1):
                                    arcs_used.append([route[r], route[r+1]])
                                arcs_used.append([route[-1], route[0]])
                                loop_length = len(route)
                                model.cbLazy(quicksum(X[a[0],a[1],t] for a in arcs_used) <= loop_length - 1)


    # Minimum number of customers that must be visited by trucks
    # minNodes = max(math.ceil(numberCustomers*numberTrucks/(numberTrucks+numberDrones)), numberCustomers - len(removal_nodes))
    """
    Sub Problem
    """
    BSP = Model()
    BSP.setParam('OutputFlag', 0)
    """
    Suproblem variables
    """
    # 1 if drone d leaves truck at node i to deliver to node c and returns to truck at node j. 0 otherwise
    Z = {(i,c,j): BSP.addVar(vtype = GRB.BINARY) for i in C for c in C for j in C}
    # Wait time of truck at customer c
    WAIT = {c: BSP.addVar(lb=0) for c in C}
    """
    Subproblem Objective Function 
    """
    BSP.setObjective(quicksum(WAIT[c] for c in C), GRB.MINIMIZE)
    """
    Subproblem constarints
    """
    # Wait time must be greater than or equal to the difference in time between drone delivery and truck delivery
    relate_wait_and_z_moving_truck = {(i,c,j):
                    BSP.addConstr(WAIT[j] >= Z[i,c,j]*((dist[i][c] + dist[c][j] - dist[i][j])/SR))
                    for i in C for c in C for j in C}
    # Must visit every node not alrady visited by truck
    # TThe right hand side of thisgets updated by the master problem solution
    visited_every_drone_node = {c:
                    BSP.addConstr(quicksum(Z[i,c,j] for i in C for j in C) == 1)
                    for c in C}
    # Number of drone trips assigned to an arc must be less than the number of drones assigned to truck T
    valid_trip = {(i,j):
                    BSP.addConstr(quicksum(Z[i,c,j] for c in C) <= numberDrones)
                    for i in C for j in C}
    # Number of drones assigned is less than total drones available
    #total_drone_count = { BSP.addConstr(quicksum(A[t] for t in T) == numberDrones) }

    """
    Master Problem
    """
    BMP = Model()
    """
    Variables
    """
    # Truck arc variables
    X = {(i,j,t): BMP.addVar(vtype=GRB.BINARY) for i in C for j in C for t in T}
    # Y used to be the longest truck route
    Y = BMP.addVar()
    # Dummy variable for assigning customers to truck arcs for drone deliveries.
    # Used to make an estimatation of theta
    DUMMY_DRONE = {(i,c,j): BMP.addVar(vtype=GRB.BINARY) for i in C for c in C for j in C}
    # Wait time associated with arc i-j
    Theta = {(i,j): BMP.addVar(lb=0) for i in C for j in C}
    """
    Objective
    """
    BMP.setObjective(Y + quicksum(Theta[i,j] for i in C for j in C), GRB.MINIMIZE)
    BMP.setParam('LazyConstraints',1)

    """
    Master Problem Constraints
    """
    # Make y Greater than or equal to every truck route.
    # As this is a minimization problem, this will force Y to be the largest trck route
    relate_x_to_y = {t:
                BMP.addConstr(quicksum(X[i,j,t] * (dist[i][j]) for i in C for j in C) <= Y)
                    for t in T}
    # Ensure that the flow of the trucks are balanced
    balance_inflow_outflow = {c:
                BMP.addConstr(quicksum(X[i,c,t] for i in C for t in T)
                    == quicksum(X[c,j,t] for j in C for t in T))
                    for c in C}
    # Every node must be visited by a truck or drone (dummy variable used here)
    must_be_visited_by_drone_or_truck = {c:
                BMP.addConstr(quicksum(X[i,c,t] for i in C for t in T) +
                    quicksum(DUMMY_DRONE[i,c,j] for i in C for j in C) == 1)
                    for c in C}
    # Ensures that any nodes we have left for the drone to deliver to are reachable when the drone limit is considered
    relate_dummy_drone_to_x_and_limit = {(i,c,j):
                BMP.addConstr(DUMMY_DRONE[i,c,j] <= quicksum(X[i,j,t]*drone_limit/(dist[i][c]+dist[c][j]) for t in T))
                    for i in C for c in C for j in C}
    # Estimation of the cost associated with the drone deliveries
    relate_dummy_drone_to_theta = {(i,j):
                BMP.addConstr(Theta[i,j] >= quicksum(DUMMY_DRONE[i,c,j] * minDist[i][j] for c in C))
                    for i in C for j in C}
    
    # Every truck must leave the depot
    root_outflow = {t:
                BMP.addConstr(quicksum(X[0,j,t] for j in C) == 1)
                    for t in T}

    # CAnnot go flow to itself
    # Truck must visit at leat the number of customers calculated earlier
    truck_visits_number_nodes = BMP.addConstr(quicksum(X[i,j,t] for i in C for j in C for t in T) >= minNodes)

    BMP.setParam('OutputFlag',1)
    cuts_added = 0
    while_counter = 0
    bestEver = 99999
    while_attempts = 0
    while True:
        while_counter += 1
        BMP.optimize(Callback)
        #print("\n\n\nPAST OPTIMIZE\n\n\n")
        # Retrieve all the truck arcs
        active_arcs = []
        for t in T:
            truck_arcs = []
            for i in C:
                for j in C:
                    if X[i,j,t].x > 0.9:
                        truck_arcs.append([i,j])
            active_arcs.append(truck_arcs)

        # Show truck route every 'show_frequency' loops:
        if show_frequency != 0:
            if while_counter % show_frequency == 0:
                plt.scatter(xVals[0], yVals[0], c='r', marker='s')
                plt.scatter(xVals[1:], yVals[1:], c='b')
                for i in C:
                    plt.annotate(i, (customers[i][0]+1, customers[i][1]+1))

                colours = ['b', 'g', 'r', 'm', 'k', 'y']
                for t in T:
                    for s in active_arcs[t]:
                        i = s[0]
                        j = s[1]
                        plt.plot([customers[i][0], customers[j][0]], [customers[i][1], customers[j][1]], c=colours[t])
                plt.draw()

        # Get the nodes visited and not visited by trucks
        visited_nodes = []
        all_truck_arcs = []
        truck_visit = []
        for t in T:
            truck_arcs = []
            for i in C:
                for j in C:
                    if X[i,j,t].x > 0.9:
                        all_truck_arcs.append([i,j])
                        if j not in visited_nodes:
                            visited_nodes.append(j)
                            truck_arcs.append([i,j])
            truck_visit.append(truck_arcs)
        not_truck_arcs = []
        for i in C:
            for j in C:
                if [i,j] not in truck_arcs and [j,i] not in truck_arcs:
                    not_truck_arcs.append([i,j])
        # Nodes to be visited by drones
        drone_nodes = []
        for c in C:
            if c not in visited_nodes:
                drone_nodes.append(c)
        # Update valid_trip constraint depending on x vals found in master problem
        for i in C:
            for j in C:
                truck_on_arc = 0
                for t in T:
                    if X[i,j,t].x > 0.9:
                        truck_on_arc = 1
                valid_trip[i,j].RHS = truck_on_arc*numberDrones
        # Update visited_every_drone_node constraint depending on the x vals found in the master problem
        for c in C:
            if c in visited_nodes:
                visited_every_drone_node[c].RHS = 0
            else:
                visited_every_drone_node[c].RHS = 1
        BSP.optimize()
        # Status is 2 if optimal solution found. 3 if infeasible
        status = BSP.Status
        if status == GRB.OPTIMAL:
            drone_allocation_cost = BSP.objVal
            #print(f'drone allocation cost: {drone_allocation_cost}')
            thetaVal = 0
            for i in C:
                for j in C:
                    thetaVal += Theta[i,j].x
            # Solution is Master + subproblem - estimation of subproblem
            solution = BMP.objVal + BSP.objVal - thetaVal

            if solution< bestEver - EPS:
                #print("Optimality cut added. New best:", solution)
                cuts_added += 1
                bestEver = solution
                if drone_allocation_cost > thetaVal + EPS:
                    BMP.addConstr(quicksum(Theta[i,j] for i in C for j in C) >=
                                drone_allocation_cost*(1 - quicksum(1 - X[a[0], a[1],t] for a in all_truck_arcs for t in T)
                                - quicksum(X[a[0], a[1], t] for a in not_truck_arcs for t in T)))
                continue
            else:
                break;
        else:
            if status == GRB.INFEASIBLE:
                #print('Infeasible')
                number_truck_arcs = len(all_truck_arcs)
                # One of the drone nodes needs the truck to deliver to it
                BMP.addConstr(quicksum(X[i, c, t] for i in C for c in drone_nodes for t in T) >= 1)
                cuts_added += 1
                #print("Infeasibility cut added")
    print("Optimal Solution:", bestEver)
    #print("Total cuts added: ", cuts_added)
    end = time.time()


    # Get truck arcs
    total_truck_dist = 0
    truck_visits = []
    active_arcs = []
    for t in T:
        truck_route = []
        for i in C:
            for j in C:
                if X[i,j,t].x > 0.9:
                    total_truck_dist += dist[i][j]
                    truck_route.append([i,j])
                    if j not in truck_visits:
                        truck_visits.append(j)
        active_arcs.append(truck_route)
    #print("Truck:", active_arcs)

    # Get drone arcs
    drone_arcs = []
    for i in C:
        if i in truck_visits:
            for c in C:
                if c not in truck_visits:
                    for j in C:
                        if j in truck_visits and j != i:
                            if Z[i,c,j].x > 0.9:
                                drone_arcs.append([i,c,j])
    #print("Drone:", drone_arcs)

    # Print final solution
    plt.scatter(xVals[0], yVals[0], c='r', marker='s')
    plt.scatter(xVals[1:], yVals[1:], c='b')


    truck_arcs_ordered = tour_nodes(0, active_arcs[0], [0])
    truck_arcs_ordered.append(0)

    fig, ax = plt.subplots()

    xAnim = []
    yAnim = []

    xDroneAnim = [[[] for _ in range(numberDrones)] for _ in range(numberCustomers)]
    yDroneAnim = [[[] for _ in range(numberDrones)] for _ in range(numberCustomers)]

    arcIndexes = [0]

    for i, customer in enumerate(truck_arcs_ordered):
        if i == len(truck_arcs_ordered) - 1:
            break
        currentCust = customers[customer]
        nextCust = customers[truck_arcs_ordered[i+1]]
        currentDrones = []
        for arc in drone_arcs:
            custs = [arc[0], arc[2]]
            if truck_arcs_ordered[i] in custs and truck_arcs_ordered[i+1] in custs:
                currentDrones.append([currentCust, customers[arc[1]], nextCust])

        dist = math.hypot((currentCust[0]-nextCust[0]), (currentCust[1]-nextCust[1]))
        
        if currentCust[0] != nextCust[0]:
            xArc = list(np.arange(currentCust[0], nextCust[0], (nextCust[0]-currentCust[0])/dist))
        else:
            xArc = [nextCust[0] for _ in range(int(dist))]
        

        if currentCust[1] != nextCust[1]:
            yArc = list(np.arange(currentCust[1], nextCust[1], (nextCust[1]-currentCust[1])/dist))
        else:
            yArc = [nextCust[1] for _ in range(int(dist))]
        

        for j, drone in enumerate(currentDrones):
            firstDroneDist = math.hypot(drone[1][0]-drone[0][0], drone[1][1]-drone[0][1])
            secondDroneDist = math.hypot(drone[2][0]-drone[1][0], drone[2][1]-drone[1][1])
            if drone[1][0] != drone[0][0]:
                firstDroneArcX = list(np.arange(drone[0][0], drone[1][0], SR*(drone[1][0]-drone[0][0])/firstDroneDist))
            else:
                firstDroneArcX = [drone[1][0] for _ in range(math.ceil(firstDroneDist/SR))]
            if drone[1][1] != drone[0][1]:
                firstDroneArcY = list(np.arange(drone[0][1], drone[1][1], SR*(drone[1][1]-drone[0][1])/firstDroneDist))
            else:
                firstDroneArcY = [drone[1][1] for _ in range(math.ceil(firstDroneDist/SR))]
            if drone[2][0] != drone[1][0]:
                secondDroneArcX = list(np.arange(drone[1][0], drone[2][0], SR*(drone[2][0]-drone[1][0])/secondDroneDist))
            else:
                secondDroneArcX = [drone[2][0] for _ in range(math.ceil(secondDroneDist/SR))]
            if drone[2][1] != drone[1][1]:
                secondDroneArcY = list(np.arange(drone[1][1], drone[2][1], SR*(drone[2][1]-drone[1][1])/secondDroneDist))
            else:
                secondDroneArcY = [drone[2][0] for _ in range(math.ceil(secondDroneDist/SR))]
        
            droneX = firstDroneArcX+secondDroneArcX
            droneY = firstDroneArcY+secondDroneArcY

            xDroneAnim[i][j] = droneX
            yDroneAnim[i][j] = droneY
        
        if numberDrones > 0:
            droneLen = [len(drone) for drone in xDroneAnim[i]]
            length = max(droneLen)
            while length > len(xArc):
                xArc.append(xArc[-1])
                yArc.append(yArc[-1])
        for x in xArc:
            xAnim.append(x)
        for y in yArc:
            yAnim.append(y)
        arcIndexes.append(len(xAnim))

        for n in range(numberDrones):
            if len(xDroneAnim[i][n]) > 0:
                while len(xDroneAnim[i][n]) < len(xArc):
                    xDroneAnim[i][n].append(nextCust[0])
                    yDroneAnim[i][n].append(nextCust[1])

    def func(i):
        #print(i)
        x = random.randint(0,100)
        y = random.randint(0,100)
        if i == 0:
            current_arc = 0
            plt.clf()
            plt.xlim([0,100])
            plt.ylim([0,100])
            plt.scatter(xVals[0], yVals[0], c='r', marker='s')
            plt.scatter(xVals[1:], yVals[1:], c='b')
        current_arc = -1
        for index in arcIndexes:
            if i >= index:
                current_arc += 1
            else:
                break
        i = int(i)
        #print(i)
        plt.plot([xAnim[i], xAnim[i+1]], [yAnim[i], yAnim[i+1]], c='b', alpha=0.5)
        for n in range(numberDrones):
            if len(xDroneAnim[current_arc][n]) > 0:
                xDroneAnimCur = xDroneAnim[current_arc][n]
                yDroneAnimCur = yDroneAnim[current_arc][n]
                if i+1 not in arcIndexes:
                    plt.plot([xDroneAnimCur[i - arcIndexes[current_arc]], xDroneAnimCur[i - arcIndexes[current_arc] + 1]], [yDroneAnimCur[i - arcIndexes[current_arc]], yDroneAnimCur[i - arcIndexes[current_arc] + 1]], c='r', linestyle='dotted', alpha=0.5)
        
    #print(f'Time taken: {end-start}')
    _animation = FuncAnimation(fig, func, frames=np.arange(0, len(xAnim)-1, 1), interval=20, repeat=True)
    #print("Animation complete")
    plt.xlim([0,100])
    plt.ylim([0,100])

    writer = PillowWriter(fps=30)
    
    if int(SR) == SR:
        SR_str = str(int(SR))
    else:
        SR_str = f'1_{int(100*(SR-1))}'

    #fileName = f'{numberCustomers}C-{numberTrucks}T-{numberDrones}D-{SR_str}SR.gif'
    if numberDrones == 0:
        fileName = f'{numberCustomers}C-{numberTrucks}T-{numberDrones}D.gif'
    else:
        fileName = f'{numberCustomers}C-{numberTrucks}T-{numberDrones}D-{SR_str}SR-{int(drone_limit)}LIM.gif'
    print(f'fileName: {fileName}')
    print(f'Time taken: {end-start}')
    print()
    _animation.save(fileName, writer=writer)
    #print("Animation saved")


    #plt.show()

    """
    for i in C:
        plt.annotate(i, (customers[i][0]+1, customers[i][1]+1))
    colours = ['b', 'g', 'r', 'm', 'k', 'y']
    for t in T:
        truck_route = active_arcs[t]
        for a in truck_route:
            i = a[0]
            j = a[1]
            plt.plot([customers[i][0], customers[j][0]], [customers[i][1], customers[j][1]], c=colours[t])
    for a in drone_arcs:
        i = a[0]
        c = a[1]
        j = a[2]
        plt.plot([customers[i][0], customers[c][0]], [customers[i][1], customers[c][1]], c='r', linestyle='dashed')
        plt.plot([customers[c][0], customers[j][0]], [customers[c][1], customers[j][1]], c='r', linestyle='dashed')

    print("Min nodes:", minNodes)
    print("Done")
    print(f'Time elapsed: {end-start}')
    plt.show()

    """


shortest_route(30,1,3,1.25,50)
xxxxxxxx

for numCustomers in range(5,30,5):
    for numDrone in range(5):
        if numDrone != 0:
            for lim in range(30, 120, 15):
                for i in range(5):
                    SR = 1 + i*0.25
                    print(f'Customers: {numCustomers}')
                    print(f'Drones: {numDrone}')
                    print(f'Drone limit: {lim}')
                    print(f'Speed Raio: {SR}')
                    shortest_route(numCustomers,1,numDrone,SR, lim)
        else:
            shortest_route(numCustomers,1,0,1,0)
