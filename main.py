# built-in
import os
import sys
import traci
from tabulate import tabulate

# own
from opt.opt import Optimization

def print_users_info(user_wt, user_tt, user_dtr):
    print_user = {
        "ID": [],
        "pick-up": [],
        "drop-off": [],
        "waiting time": [],
        "travel time": [],
        "detour ratio": []
    }

    for uid in traci.person.getTaxiReservations(0):
        pid = uid.persons[0]
        print_user["ID"].append(pid)
        print_user["pick-up"].append(uid.fromEdge)
        print_user["drop-off"].append(uid.toEdge)
        if uid.state == 4:
            print_user["waiting time"].append(user_wt[pid])
            print_user["travel time"].append(user_tt[pid])
            print_user["detour ratio"].append(user_dtr[pid])

    print("=== Requests ===")
    print(tabulate(print_user, headers="keys", tablefmt="fancy_grid", missingval="N/A"))

def print_veh_info(veh_users, veh_route):
    print_veh = {
        "ID": [],
        "users": [],
        "path": [],
        "via": []
    }

    for vid in opt.vehicles:
        print_veh["ID"].append(vid)
        print_veh["users"].append(veh_users[vid])
        print_veh["path"].append(veh_route[vid])
        if traci.vehicle.getRoadID(vid):
            print_veh["via"].append(traci.vehicle.getRoadID(vid))
        else:
            print_veh["via"].append("currently booking")

    print("=== Vehicles ===")
    print(tabulate(print_veh, headers="keys", tablefmt="fancy_grid", missingval="N/A"))

if 'SUMO_HOME' in os.environ:
    # sumo environment path
    sys.path.append(os.path.join(os.environ['SUMO_HOME'], 'tools'))

    # set sumo configuration
    sumoBinary = "/usr/bin/sumo-gui"
    sumoCmd = [sumoBinary, "-c", "./env/large_scale/osm.sumocfg"]

    # simulation start
    traci.start(sumoCmd)

    # parameters
    dtr_thr = 3.0               # threshold of detour

    # optimization class
    opt = Optimization()

    result_vid_route = dict()

    for step in range(350):
        traci.simulationStep()

        # print the current simulation time
        print(f"Current simulation time: {int(traci.simulation.getTime())} seconds")
        # add vehicles in optimization model
        for vid in traci.vehicle.getIDList():
            route = list(traci.vehicle.getRoute(vid))
            if len(route) == 1:
                opt.add_vehicles(vid, route)
            else:
                opt.add_vehicles(vid, result_vid_route[vid])

        # add users in optimization model
        # reservation state
        # 0: return all reservations regardless of state
        # 1: return only new reservations
        # 2: return reservations already retrieved
        # 4: return reservations that have been assigned to a taxi
        # 8: return reservations that have been picked up
        for uid in traci.person.getTaxiReservations(0):
            if uid.state == 4:
                continue
            else:
                pu, do = traci.person.getEdges(uid.persons[0])
                opt.add_users(uid.persons[0], uid.id, pu, do)

        if len(opt.vehicles) == 0 or len(opt.users) == 0:
            continue
        else:
            result_vid_route, result_vid_users, result_user_wt, result_user_tt, result_user_dtr = opt.opt(dtr_thr)
            print_users_info(result_user_wt, result_user_tt, result_user_dtr)
            print_veh_info(result_vid_users, result_vid_route)
    traci.close()