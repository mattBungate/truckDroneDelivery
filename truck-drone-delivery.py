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
import matplotlib.pyplot as plt
from matplotlib.animation import PillowWriter
from matplotlib.animation import FuncAnimation
import csv
import time

# File imports
import functions as F
import animate as A


seedNum = 1
random.seed(seedNum)
EPS = 0.0001
# numberCustomers - Number of customers that require a delivery
# numberTrucks - Number of trucks that are delivering
# numberDrones - Number of drones that are delivering
# SR - Speed Ratio - Ratio of the speed of the drones to the truck
# drone_limit - Maximum distance a drone can travel before returning to a truck
# 
# Creates animation and saves it as a gif file
# 
def shortest_route(numberCustomers, numberTrucks, numberDrones, SR, droneLimit, csvPath='Routes.csv', showAnimation=True):
    csvInput = [numberCustomers, numberTrucks, numberDrones, SR, droneLimit, seedNum]
    drone_limit = float(droneLimit)
    # Sets
    C = range(numberCustomers)
    T = range(numberTrucks)
    D = range(numberDrones)

    # Start timer
    startTime = time.time()

    # Create the instance
    customers, dist, minDist = F.instanceGeneration(numberCustomers, numberTrucks, 'r')

    valid_drone_nodes = F.valid_drone_routes(dist, drone_limit)

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
    #F.createInitialPlot(customers)

    # Callback function called by the master problem to eliminate subtours
    def Callback(model, where):
        if where == GRB.Callback.MIPSOL:
            XV = model.cbGetSolution(X)
            for t in T:
                active_arcs = F.getTruckArcsCallback(XV, numberCustomers, t)
                for c in range(numberCustomers):
                    for a in active_arcs:
                        if c == a[0]:
                            route = F.tour_nodes(c, active_arcs, [c])
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
        # Retrieve all the truck arcs
        active_arcs = []
        for t in T:
            truck_arcs = []
            for i in C:
                for j in C:
                    if X[i,j,t].x > 0.9:
                        truck_arcs.append([i,j])
            active_arcs.append(truck_arcs)

        # Get the nodes visited and not visited by trucks
        visited_nodes, all_truck_arcs, truck_visit = F.getTruckArcs(X, numberCustomers, numberTrucks)

        not_truck_arcs = []
        for i in C:
            for j in C:
                if [i,j] not in truck_visit and [j,i] not in truck_visit:
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
            thetaVal = 0
            for i in C:
                for j in C:
                    thetaVal += Theta[i,j].x

            # Solution is Master + subproblem - estimation of subproblem
            solution = BMP.objVal + BSP.objVal - thetaVal

            if solution< bestEver - EPS:
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
                number_truck_arcs = len(all_truck_arcs)
                # One of the drone nodes needs the truck to deliver to it
                BMP.addConstr(quicksum(X[i, c, t] for i in C for c in drone_nodes for t in T) >= 1)
                cuts_added += 1
                
    print("Optimal Solution:", bestEver)
    endTime = time.time()

    csvInput.append(endTime-startTime)

    # Get truck arcs
    total_truck_dist = 0
    truck_visits, _, active_arcs = F.getTruckArcs(X,numberCustomers, numberTrucks)

    truck_arcs_ordered = F.tour_nodes(0, active_arcs[0], [0])
    truck_arcs_ordered.append(0)

    csvInput.append('Truck')
    for node in truck_arcs_ordered:
        csvInput.append(node)

    # Get drone arcs
    drone_arcs = F.getDroneArcs(truck_visits, Z, numberCustomers)

    csvInput.append('Drones')
    for arc in drone_arcs:
        csvInput.append(f'{arc[0]}-{arc[1]}-{arc[2]}')

    # Write solution to csv file
    file = open(csvPath, 'w')
    csvWriter = csv.writer(file)
    csvWriter.writerow(csvInput)
    file.close()

    if showAnimation:
        A.animateCSV(csvPath, customers)

shortest_route(15,1,3,1.25,50)
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
