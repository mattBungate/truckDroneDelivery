# truckDroneDelivery
Finds the optimal way for trucks and drones to delivery packages to randomly generate customers.
This method utilizes Gurobi as an optimisation tool, and applies Benders' Decomposition in an attempt to manage the combinatorial nature of the problem. 

## truck-drone-delivery.py
Contains the mathematical formulation and calculation code done through gurobi. Writes solution to csv file

## functions.py
Contains all helper functions used in truck-drone-delivery.py to make that file more readable

## animate
Reads the solution from a csv file and creates an animation for that solution. 
