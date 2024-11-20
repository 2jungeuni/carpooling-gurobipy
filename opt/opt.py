# built-in
import os
import sys
import numpy as np

# solver
import gurobipy as gp
from gurobipy import *

# sumo sim
import traci

# optimization status dictionary
status_dict = {
    1: "loaded",
    2: "optimal",
    3: "infeasible",
    4: "infeasible and unbounded",
    5: "unbounded",
    6: "cut off",
    7: "iteration limit",
    8: "node limit",
    9: "time limit",
    10: "solution limit",
    11: "interrupted",
    12: "numeric",
    13: "suboptimal",
    14: "in progress",
    15: "user objective limit",
    16: "work limit",
    17: "memory limit"
}

class Optimization:
    def __init__(self):
        super(Optimization, self).__init__()
        self.vehicles = set()
        self.users = set()

        # vehicles info
        self.vid_loc = dict()
        self.vid_passengers = dict()
        self.vid_fixed_stops = dict()

        # passengers info
        self.pid_pu = dict()
        self.pid_do = dict()
        self.pid_rid = dict()
        self.pid_depart_time = dict()
        self.pid_waiting_time = dict()
        self.pid_shortest_time = dict()

        # distance matrix
        self.distance = {0: {}}

        # result
        self.result_vid_users = dict()
        self.result_vid_route = dict()
        self.result_user_wt = dict()
        self.result_user_tt = dict()
        self.result_user_dtr = dict()

    def get_cost(self, frm, to):
        if frm not in self.distance.keys():
            self.distance[frm] = {}
        if to not in self.distance[frm].keys():
            if frm == 0 or to == 0:
                self.distance[frm][to] = 0
            else:
                route = traci.simulation.findRoute(frm, to)
                tt = route.travelTime
                if len(route.edges) > 0:
                    self.distance[frm][to] = tt
                else:
                    self.distance[frm][to] = math.inf
        return self.distance[frm][to]

    def add_vehicles(self, vid, stops):
        depart_pos = stops[0]
        if vid not in self.vehicles:
            self.vehicles.add(vid)
            self.vid_loc[vid] = depart_pos
            self.vid_passengers[vid] = set()
            self.result_vid_users[vid] = []
            self.result_vid_route[vid] = []
        self.set_stops(vid, stops)

    def add_users(self, pid, rid, pu, do):
        if pid not in self.users:
            self.users.add(pid)
            self.pid_pu[pid] = pu
            self.pid_do[pid] = do
            self.pid_rid[pid] = rid
            self.pid_depart_time[pid] = traci.simulation.getTime()
            self.pid_shortest_time[pid] = self.get_cost(pu, do)
            self.pid_waiting_time[pid] = math.inf

    def set_stops(self, vid, stops):
        self.vid_fixed_stops[vid] = stops

    def opt(self, detour):
        m = gp.Model()
        m.Params.outputFlag = False

        # idx -> stop
        # for artificial location
        idx_stop = {0: (0, 0)}
        stop_idx = {(0, 0): 0}

        idx = 1
        departs = []
        pickups = []
        dropoffs = []
        capacity = {}

        # vehicles
        vehs = [vid for vid in traci.vehicle.getIDList()
                if traci.vehicle.getPersonCapacity(vid) > len(self.vid_passengers[vid])]

        for vid in vehs:
            # add positions of vehicles in opt system
            stops = self.vid_fixed_stops[vid]
            departs.append(stops[0])
            capacity[vid] = traci.vehicle.getPersonCapacity(vid) - len(self.vid_passengers[vid])
            for stop in stops:
                idx_stop[idx] = (stop, vid)
                stop_idx[(stop, vid)] = idx
                idx += 1

        # add users in opt system
        for uid in self.users:
            pu = self.pid_pu[uid]
            do = self.pid_do[uid]

            # add pick-up positions of users in opt system
            idx_stop[idx] = (pu, uid)
            stop_idx[(pu, uid)] = idx
            pickups.append(idx)
            idx += 1

            # add drop-off positions of users in opt system
            idx_stop[idx] = (do, uid)
            stop_idx[(do, uid)] = idx
            dropoffs.append(idx)
            idx += 1

        # number of stops
        n = len(idx_stop)
        # number of vehicles
        nv = len(vehs)

        # visit or not
        p = {}
        for i in range(n):
            for v in range(nv):
                if i in pickups:
                    p[(i, v)] = -1
                else:
                    p[(i, v)] = 0

        # visit cost
        dist = {}
        for i in range(n):
            for j in range(n):
                if i != j:
                    for k in range(nv):
                        dist[(i, j, k)] = -1 * self.get_cost(idx_stop[i][0], idx_stop[j][0])

        # set decision variables of opt system
        e_vars = m.addVars(dist.keys(), obj=dist, vtype=GRB.BINARY, name="e")
        p_vars = m.addVars(p.keys(), obj=p, vtype=GRB.BINARY, name="p")
        s_vars = m.addVars(np.arange(1, n + 1), lb=1, ub=n, vtype=GRB.INTEGER, name="s")

        # If a location is visited, it must have one incoming edge and one outgoing edge.
        for i in range(n):
            if i != 0:
                for v in range(nv):
                    m.addConstr(e_vars.sum(i, "*", v) == p_vars[i, v])
                    m.addConstr(e_vars.sum("*", i, v) == p_vars[i, v])
                m.addConstr(p_vars.sum(i, "*") <= 1)

        # Constraints for artificial location
        m.addConstr(p_vars.sum(0, "*") == nv)
        for v in range(nv):
            m.addConstr(e_vars.sum("*", 0, v) == 1)
            m.addConstr(e_vars.sum(0, "*", v) == 1)

            # connect artificial location with initial position of vehicle
            m.addConstr(e_vars[(0, stop_idx[(self.vid_loc[vehs[v]], vehs[v])], v)] == 1)

            # capacity limits
            m.addConstr(gp.quicksum(-1 * p[i, v] * p_vars[i, v] for i in range(n)) <= capacity[vehs[v]])

            # pickup-dropoff pairs
            for user in self.users:
                m.addConstr(p_vars[stop_idx[(self.pid_pu[user], user)], v] == p_vars[stop_idx[(self.pid_do[user], user)], v])

        # Fix stops where are already visited
        for v_idx, vid in enumerate(vehs):
            stops = self.vid_fixed_stops[vid]
            for s_idx, stop in enumerate(stops):
                m.addConstr(p_vars[stop_idx[(stop, vid)], v_idx] == 1)
                if s_idx != len(stops) - 1:
                    m.addConstr(e_vars[stop_idx[(stop, vid)], stop_idx[(stops[s_idx + 1], vid)], v_idx] == 1)
                    # m.addConstr(s_vars[stop_idx[(stop, vid)]] + 1 <= s_vars[stop_idx[(stops[s_idx + 1], vid)]])

        # sequences
        m.addConstrs(s_vars[i] <= s_vars[j] + n * (1 - e_vars[i, j, k]) - 1
                     for i, j, k in e_vars.keys() if i != 0 and j != 0)
        m.addConstrs(s_vars[stop_idx[(self.pid_pu[user], user)]] + 1 <= s_vars[stop_idx[(self.pid_do[user], user)]]
                     for user in self.users)

        # subtour elimination
        def subtourlim(model, where):
            if where == GRB.Callback.MIPSOL:
                # make a list of edges selected in the solution
                vals = model.cbGetSolution(model._vars)
                selected = gp.tuplelist((i, j, k) for i, j, k in model._vars.keys() if vals[i, j, k] > 0.5)

                # find the shortest cycle in the selected edge list
                tour = subtour(selected)
                for v in range(nv):
                    if tour[v]:
                        for tv in tour[v]:
                            if len(tv) < n:
                                # add subtour elimination constraint for every pair of cities in tour
                                model.cbLazy(gp.quicksum(model._vars[i, j, v] for i, j in itertools.permutations(tv, 2))
                                             <= len(tv) - 1)

        def subtour(edges, exclude_depot=True):
            cycle = [[] for v in range(nv)]

            for v in range(nv):
                unvisited = list(np.arange(0, n))

                while unvisited:  # true if list is non-empty
                    this_cycle = []
                    neighbors = unvisited

                    while neighbors:
                        current = neighbors[0]
                        this_cycle.append(current)
                        unvisited.remove(current)
                        neighbors = [j for i, j, k in edges.select(current, '*', '*') if (j in unvisited) and (k == v)]

                    if len(this_cycle) > 1:
                        if exclude_depot:
                            if not (0 in this_cycle):
                                cycle[v].append(this_cycle)
            return cycle

        m._vars = e_vars
        m._dvars = p_vars
        m._ddvars = s_vars
        m.Params.lazyConstraints = 1
        m.optimize(subtourlim)

        print(f"Solved ({status_dict[m.status]})")

        # non-optimal
        if m.status != 2:
            if m.status == 3:
                m.computeIIS()
                if os.path.exists("./result"):
                    m.write("./result/model.ilp")
                else:
                    os.mkdir("./result")
                    m.write("./result/model.ilp")

            sys.exit("There is no solution. Check ./opt_result/model.ilp.")

        # optimal
        e_vals = m.getAttr("x", e_vars)
        p_vals = m.getAttr("x", p_vars)

        sol = {}
        for v in range(nv):
            sol[v] = {}
        for i, j, k in e_vals.keys():
            if e_vals[i, j, k] > 0.5:
                sol[k][i] = j

        # summary results
        for v in range(nv):
            i = 0
            j = sol[v][i]
            sol_route = []
            sol_acceptance = dict()
            while j != 0:
                sol_route.append(idx_stop[j][0])
                j = sol[v][j]
                if j in pickups:
                    sol_acceptance[idx_stop[j][1]] = math.inf

            while not all(value < detour for value in sol_acceptance.values()):
                for pid in sol_acceptance.keys():
                    pu_index = sol_route.index(self.pid_pu[pid])
                    do_index = sol_route.index(self.pid_do[pid])
                    dtr_tt = 0
                    for idx in range(pu_index, do_index):
                        dtr_tt += traci.simulation.findRoute(sol_route[idx], sol_route[idx+1]).travelTime
                        sol_acceptance[pid] = dtr_tt / self.pid_shortest_time[pid]
                # sort
                sol_acceptance = {k: v for k, v in
                                  sorted(sol_acceptance.items(), key=lambda item: item[1], reverse=True)}
                for pid in sol_acceptance.keys():
                    # over detour ratio
                    if sol_acceptance[pid] > detour:
                        sol_route.remove(self.pid_pu[pid])
                        sol_route.remove(self.pid_do[pid])
                        sol_acceptance.pop(pid)
                        break

            self.result_vid_users[vehs[v]] += list(sol_acceptance.keys())
            self.result_vid_route[vehs[v]] = list(sol_route)
            if len(sol_route) > 1:
                detail_route = []
                for idx in range(len(sol_route) - 1):
                    detail_route += list(traci.simulation.findRoute(sol_route[idx], sol_route[idx + 1]).edges)
                traci.vehicle.setRoute(vehs[v], detail_route)

            for pid in sol_acceptance.keys():
                traci.vehicle.dispatchTaxi(vehs[v], self.pid_rid[pid])
                self.users.remove(pid)

                # calculate waiting time
                pu_index = sol_route.index(self.pid_pu[pid])
                self.result_user_wt[pid] = 0
                for i in range(pu_index):
                    self.result_user_wt[pid] += traci.simulation.findRoute(sol_route[i], sol_route[i+1]).travelTime
                self.result_user_tt[pid] = sol_acceptance[pid] * self.pid_shortest_time[pid]
                self.result_user_dtr[pid] = sol_acceptance[pid]

        return self.result_vid_route, self.result_vid_users, self.result_user_wt, self.result_user_tt, self.result_user_dtr