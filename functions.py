import math
import random as rand
import csv
import matplotlib.pyplot as plt


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
        for i in range(numberCustomers):
            minDist.append([])
            for j in range(numberCustomers):
                minDist[i].append(max(min(dist[i][c] + dist[c][j] - dist[i][j] for c in range(numberCustomers)), 0))
        return customers, dist, minDist
    else:
    # Randomly generated customers
    # Set the node to the centre of the problem
        customers.append([50,50])
        for c in range(numberCustomers - 1):
            xVal = rand.randint(0, 100)
            yVal = rand.randint(0, 100)
            node = (xVal, yVal)
            customers.append(node)
        dist = distGen(customers, numberCustomers, [])
        minDist = [] 
        for i in range(numberCustomers):
            minDist.append([])
            for j in range(numberCustomers):
                minDist[i].append(max(min(dist[i][c] + dist[c][j] - dist[i][j] for c in range(numberCustomers)), 0))
        return customers, dist, minDist


def valid_drone_routes(dist, drone_limit):
    C = range(len(dist))
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

def createInitialPlot(customers):
    xVals = []
    yVals = []
    for i in customers:
        xVals.append(i[0])
        yVals.append(i[1])
    plt.scatter(xVals[0], yVals[0], c='r', marker='s')
    plt.scatter(xVals[1:], yVals[1:], c='b')
    plt.xlim([0,100])
    plt.ylim([0,100])
    plt.show(block=False)

def getTruckArcsCallback(values, numberCustomers, t):
    active_arcs = []
    for i in range(numberCustomers):
        for j in range(numberCustomers):
            if values[i,j,t] > 0.9:
                active_arcs.append([i,j])
    return active_arcs

def getTruckArcs(values, numberCustomers, numberTrucks):
    visited_nodes = []
    all_truck_arcs = []
    truck_visit = []
    for t in range(numberTrucks):
        truck_arcs = []
        for i in range(numberCustomers):
            for j in range(numberCustomers):
                if values[i,j,t].x > 0.9:
                    all_truck_arcs.append([i,j])
                    if j not in visited_nodes:
                        visited_nodes.append(j)
                        truck_arcs.append([i,j])
        truck_visit.append(truck_arcs)
    return visited_nodes, all_truck_arcs, truck_visit

def getDroneArcs(truck_visits, Z, numberCustomers):
    drone_arcs = []
    C = range(numberCustomers)
    for i in C:
        if i in truck_visits:
            for c in C:
                if c not in truck_visits:
                    for j in C:
                        if j in truck_visits and j != i:
                            if Z[i,c,j].x > 0.9:
                                drone_arcs.append([i,c,j])
    return drone_arcs

