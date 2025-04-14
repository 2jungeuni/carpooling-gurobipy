# built-in
import os
import sys
import numpy as np

# solver
import gurobipy as gp
from gurobipy import *

# sumo sim
import traci

# Dictionary to interpret Gurobi optimization status codes
# Gurobi 최적화 상태 코드와 그 의미
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
    """
    This class handles vehicle-user assignment and routing optimization using SUMO (Traci) and Gurobi.

    SUMO와 Gurobi를 사용해 차량-사용자 매칭 및 경로를 최적화하는 클래스.
    """
    def __init__(self):
        super(Optimization, self).__init__()
        # Set to store active vehicles and users
        # 차량, 사용자 저장
        self.vehicles = set()
        self.users = set()

        # Vehicle info: location, passengers, stops that are already fixed
        # 차량별 위치, 이미 탑승 중인 승객, 미리 지정된 stops 정보
        self.vid_loc = dict()
        self.vid_passengers = dict()
        self.vid_fixed_stops = dict()

        # Passenger info: pick-up, drop-off, reservation ID, departure time, waiting time, shortest travel time
        # 사용자별 pick-up / drop-off / reservation id / shortest time 등
        self.pid_pu = dict()
        self.pid_do = dict()
        self.pid_rid = dict()
        self.pid_depart_time = dict()
        self.pid_waiting_time = dict()
        self.pid_shortest_time = dict()

        # Distance (time) cache: for quick lookup of frm->to travel time
        # frm -> to 로 이동 시 소요 시간 / 경로를 빠르게 찾기 위한 distance 캐싱
        self.distance = {0: {}}

        # Result placeholders
        # 최적화 결과 저장
        self.result_vid_users = dict()
        self.result_vid_route = dict()
        self.result_user_wt = dict()
        self.result_user_tt = dict()
        self.result_user_dtr = dict()

    def get_cost(self, frm, to):
        """
        Returns the travel time from `frm` to `to`, using cached results if available.

        frm -> to 이동 시 결리는 시간을 캐싱하여 반환한다.
        """
        if frm not in self.distance.keys():
            self.distance[frm] = {}
        if to not in self.distance[frm].keys():
            # If frm == 0 or to == 0, treat them as an 'artifical' node
            # frm == 0 또는 to == 0은 인위적인(artificial) 노드
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
        """
        Register a new vehicle (vid) and its initial or known route (stops).

        새로운 차량(vid) 및 초기 경로(stops)를 등록한다.
        """
        depart_pos = stops[0]
        if vid not in self.vehicles:
            self.vehicles.add(vid)
            self.vid_loc[vid] = depart_pos
            self.vid_passengers[vid] = set()
            self.result_vid_users[vid] = []
            self.result_vid_route[vid] = []
        self.set_stops(vid, stops)

    def add_users(self, pid, rid, pu, do):
        """
        Register a new uwer (pid) with pick-up, drop-off, and reservation ID (rid).

        새로운 사용자(pid)와 해당 사용자의 pick-up, drop-off 위치, 그리고 예약 id(rid) 등록.
        """
        if pid not in self.users:
            self.users.add(pid)
            self.pid_pu[pid] = pu
            self.pid_do[pid] = do
            self.pid_rid[pid] = rid
            self.pid_depart_time[pid] = traci.simulation.getTime()
            self.pid_shortest_time[pid] = self.get_cost(pu, do)
            self.pid_waiting_time[pid] = math.inf

    def set_stops(self, vid, stops):
        """
        Store any pre-determined intermediate stops for vehicle vid.

        차량(vid)에 대해 이미 확정된 중간 정차 지점(stops)를 저장한다.
        """
        self.vid_fixed_stops[vid] = stops

    def opt(self, detour):
        """
        Build and solve a Gurobi optimization model for the vehicle-user assignment problem.

        Gurobi 최적화 모델을 생성해 차량-사용자 배정 문제를 푼다.
        """
        m = gp.Model()
        m.Params.outputFlag = False

        # artificial node (0) 설정
        idx_stop = {0: (0, 0)}
        stop_idx = {(0, 0): 0}

        idx = 1
        departs = []
        pickups = []
        dropoffs = []
        capacity = {}

        # Only cosider vehicles that still have remaining passenger capacity
        # 현재 시뮬레이션 탑승 정원(capacity)이 남아 있는 차량만 고려
        vehs = [vid for vid in traci.vehicle.getIDList()
                if traci.vehicle.getPersonCapacity(vid) > len(self.vid_passengers[vid])]

        # 1) Register vehicle stops in the optimization model
        # 1) 차량이 정차 지점 등록
        for vid in vehs:
            # add positions of vehicles in opt system
            stops = self.vid_fixed_stops[vid]
            departs.append(stops[0])
            capacity[vid] = traci.vehicle.getPersonCapacity(vid) - len(self.vid_passengers[vid])
            for stop in stops:
                idx_stop[idx] = (stop, vid)
                stop_idx[(stop, vid)] = idx
                idx += 1

        # 2) Reigster user pick-up / drop-off points
        # 2) 사용자 픽업 / 드롭오프 지점 등록
        for uid in self.users:
            pu = self.pid_pu[uid]
            do = self.pid_do[uid]

            # pick-up
            idx_stop[idx] = (pu, uid)
            stop_idx[(pu, uid)] = idx
            pickups.append(idx)
            idx += 1

            # dropoff
            idx_stop[idx] = (do, uid)
            stop_idx[(do, uid)] = idx
            dropoffs.append(idx)
            idx += 1

        n = len(idx_stop)       # 정차 지점 (노드) 총 개수 (total number of stops (nodes))
        nv = len(vehs)          # 차량 총 개수 (total number of vehicles)

        # For visiting cost (distance) and feasibility checks
        # 방문 여부, 거리 dist를 계산할 딕셔너리
        p = {}
        for i in range(n):
            for v in range(nv):
                # If it's a pickup node, we give a negative cost for p_vars (to encourage picking up)
                # 픽업 지점인 경우 비용을 -1로 두어 방문하는 것이 이득이 되도록 설계
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

        # Decision variables: e_vars(i, j, k) / p_vars(i, k) / s_vars(i)
        # e_vars: whether vehicle k visits edge i->j
        # p_vars: whether vehicle k visits node i
        # s_vars: order in which node i is visited
        # 의사결정변수 추가: e_vars(i, j, k) / p_vars(i, k) / s_vars(i)
        # e_vars: i->j 경로를 차량 k가 방문하는지 여부
        # p_vars: 노드 i를 차량 k가 방문하는지 여부
        # s_vars: 노드 방문 순서
        e_vars = m.addVars(dist.keys(), obj=dist, vtype=GRB.BINARY, name="e")
        p_vars = m.addVars(p.keys(), obj=p, vtype=GRB.BINARY, name="p")
        s_vars = m.addVars(np.arange(1, n + 1), lb=1, ub=n, vtype=GRB.INTEGER, name="s")

        # 1) If a node i is visited, in-degree = out-degree 1 for that node
        # 1) 방문 제약: 방문하는 노드는 in-degree = out-degree = 1
        for i in range(n):
            if i != 0:  # artificial node (0)는 예외
                for v in range(nv):
                    m.addConstr(e_vars.sum(i, "*", v) == p_vars[i, v])
                    m.addConstr(e_vars.sum("*", i, v) == p_vars[i, v])
                # A node can be served by at most one vehicle
                # 한 노드는 최대 1대의 차량만 방문 가능
                m.addConstr(p_vars.sum(i, "*") <= 1)

        # 2) Artificial node constraints & capacity constraints
        # 2) Artificial node(0)와 차량 출발지 연결 및 capacity 제약
        m.addConstr(p_vars.sum(0, "*") == nv)
        for v in range(nv):
            m.addConstr(e_vars.sum("*", 0, v) == 1)
            m.addConstr(e_vars.sum(0, "*", v) == 1)

            # Connect artificial node(0) with the vehicle's starting position
            # 차량 v와 artificial node(0)를 연결
            m.addConstr(e_vars[(0, stop_idx[(self.vid_loc[vehs[v]], vehs[v])], v)] == 1)

            # Capacity limit for vehicle v
            # 수용 인원(capacity) 제약
            m.addConstr(gp.quicksum(-1 * p[i, v] * p_vars[i, v] for i in range(n)) <= capacity[vehs[v]])

            # If we pick up a user, we must drop off the same user
            # 픽업 / 드롭오프 쌍
            for user in self.users:
                m.addConstr(p_vars[stop_idx[(self.pid_pu[user], user)], v] == p_vars[stop_idx[(self.pid_do[user], user)], v])

        # 3) Already-fixed routes: keep them as is
        # 3) 기존(이미 확정) 경로는 고정
        for v_idx, vid in enumerate(vehs):
            stops = self.vid_fixed_stops[vid]
            for s_idx, stop in enumerate(stops):
                m.addConstr(p_vars[stop_idx[(stop, vid)], v_idx] == 1)
                if s_idx != len(stops) - 1:
                    # Fix edge from current stop to the next
                    # 다음 스탑으로의 edge 고정
                    m.addConstr(e_vars[stop_idx[(stop, vid)], stop_idx[(stops[s_idx + 1], vid)], v_idx] == 1)

        # 4) Sequence constraints
        # 4) 시퀀스(순서) 제약
        m.addConstrs(s_vars[i] <= s_vars[j] + n * (1 - e_vars[i, j, k]) - 1
                     for i, j, k in e_vars.keys() if i != 0 and j != 0)
        m.addConstrs(s_vars[stop_idx[(self.pid_pu[user], user)]] + 1 <= s_vars[stop_idx[(self.pid_do[user], user)]]
                     for user in self.users)

        # 5) Subtour elimination using lazy callback
        # 5) subtour(사이클) 제거를 위한 lazy callback
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

        # 최적화 결과 상태 출력
        print(f"Solved ({status_dict[m.status]})")

        # (1) If infeasible or no solution
        # (1) infeasible 등 해 찾기 실패 시
        if m.status != 2:
            if m.status == 3:
                # Compute and save IIS
                # infeasible한 경우 IIS(불가능성 진단) 파일을 저장
                m.computeIIS()
                if os.path.exists("./result"):
                    m.write("./result/model.ilp")
                else:
                    os.mkdir("./result")
                    m.write("./result/model.ilp")

            sys.exit("There is no solution. Check ./opt_result/model.ilp.")

        # (2) If optimal
        # (2) optimal 해 찾은 경우
        e_vals = m.getAttr("x", e_vars)
        p_vals = m.getAttr("x", p_vars)

        # Construct the solution for each vehicle v
        # solution에 따라 각 차량 v의 경로(sol[v]) 저장
        sol = {}
        for v in range(nv):
            sol[v] = {}
        for i, j, k in e_vals.keys():
            if e_vals[i, j, k] > 0.5:
                sol[k][i] = j

        # Rebuild route for each vehicle
        # 차량별 route 재구성
        for v in range(nv):
            i = 0
            j = sol[v][i]
            sol_route = []
            sol_acceptance = dict()

            # Start from artificial node(0), keep following edges until reaching 0 again
            # 인위적 노드(0)에서 출발 -> 0이 아닌 노드를 따라 경로 구성
            while j != 0:
                sol_route.append(idx_stop[j][0])
                j = sol[v][j]
                if j in pickups:
                    sol_acceptance[idx_stop[j][1]] = math.inf

            # Compute detour ratios and remove users whose ratio exceeds the threshold
            # 우회 비율(detour ratio) 계산 및 초과 사용자 제거
            while not all(value < detour for value in sol_acceptance.values()):
                for pid in sol_acceptance.keys():
                    pu_index = sol_route.index(self.pid_pu[pid])
                    do_index = sol_route.index(self.pid_do[pid])
                    dtr_tt = 0
                    for idx in range(pu_index, do_index):
                        dtr_tt += traci.simulation.findRoute(sol_route[idx], sol_route[idx+1]).travelTime
                        # Detour ratio = time in this route / shortest time
                        # 현재 루트로 계산한 사용자 우회 비율 = 실제 소요 시간 / 최단 시간
                        sol_acceptance[pid] = dtr_tt / self.pid_shortest_time[pid]

                # Sort by descending ration to remove the worst offender first
                # detour ratio가 큰 사용자부터 제거
                sol_acceptance = {k: v for k, v in
                                  sorted(sol_acceptance.items(), key=lambda item: item[1], reverse=True)}
                for pid in sol_acceptance.keys():
                    if sol_acceptance[pid] > detour:
                        sol_route.remove(self.pid_pu[pid])
                        sol_route.remove(self.pid_do[pid])
                        sol_acceptance.pop(pid)
                        break

            # Store results
            # 결과 저장
            self.result_vid_users[vehs[v]] += list(sol_acceptance.keys())
            self.result_vid_route[vehs[v]] = list(sol_route)

            # Set the actual routein SUMO using edges
            # SUMO에 실제 경로 설정 (edge 기반)
            if len(sol_route) > 1:
                detail_route = []
                for idx in range(len(sol_route) - 1):
                    detail_route += list(traci.simulation.findRoute(sol_route[idx], sol_route[idx + 1]).edges)
                traci.vehicle.setRoute(vehs[v], detail_route)

            # Dispatch users
            # 배정된 사용자들에 대해 waiting time, travel time, detour ratio 등 계산
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