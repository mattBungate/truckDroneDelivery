import matplotlib.pyplot as plt
import math
import numpy as np
import random 
from matplotlib.animation import PillowWriter
from matplotlib.animation import FuncAnimation
import csv


# Print final solution
def animate(numberCustomers, numberDrones, numberTrucks, truck_arcs_ordered, drone_arcs, customers, SR, drone_limit):
    xVals = []
    yVals = []
    for customer in customers:
        xVals.append(customer[0])
        yVals.append(customer[1])
    plt.scatter(xVals[0], yVals[0], c='r', marker='s')
    plt.scatter(xVals[1:], yVals[1:], c='b')

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
    print(f'Animation saved as: {fileName}')
    print()
    _animation.save(fileName, writer=writer)


def animateCSV(csvPath, customers):
    with open(csvPath, 'r') as file:
        csvreader = csv.reader(file)
        for row in csvreader:
            numberCustomers = int(row[0])
            numberTrucks = int(row[1])
            numberDrones = int(row[2])
            SR = float(row[3])
            drone_limit = int(row[4])
            state = 'Initial vals'
            truck_arcs_ordered = []
            drone_arcs = []
            for val in row:
                if val == 'Truck':
                    state = 'Truck'
                    continue
                if val == 'Drones':
                    state = 'Drones'
                    continue
                if state == 'Truck':
                    truck_arcs_ordered.append(val)
                if state == 'Drones':
                    nodes =  val.split('-')
                    drone_arcs.append([int(node) for node in nodes])

            truck_arcs = [int(node) for node in truck_arcs_ordered] 
            animate(numberCustomers, numberDrones, numberTrucks, truck_arcs, drone_arcs, customers, SR, drone_limit)


print([i for i in range(1)])