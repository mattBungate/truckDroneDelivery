import formulation as FORMULATION

print("Hello. We just need a few pieces of information to create you're animation")
numberCustomers = int(input("Number Customers: "))
numberDrones = int(input("Number Drones: "))
SR = float(input("Speed ratio of drones compared to trucks: "))
droneLimit = int(input("Limit drones can travel in one trip: "))

print("Thank you. Now calculating you're solution")
FORMULATION.shortest_route(numberCustomers, 1, numberDrones, SR, droneLimit)

