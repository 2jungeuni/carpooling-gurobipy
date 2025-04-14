# built-in
import os
import sys
import traci
from tabulate import tabulate

# own
from opt.opt import Optimization

def print_users_info(user_wt, user_tt, user_dtr):
    """
    Display user (taxi reservation) information from the simulation in tabular form.
    :param user_wt: A dictionary of waiting times per user
    :param user_tt: A dictionary of travel times per user
    :param user_dtr: A dictionary of detour ratios per user

    시뮬레이션 상의 사용자(택시 예약) 정보를 표 형태로 출력한다.
    :param user_wt: 각 사용자별 waiting time
    :param user_tt: 각 사용자별 travel time
    :param user_dtr: 각 사용자별 detour ratio
    """
    print_user = {
        "ID": [],
        "pick-up": [],
        "drop-off": [],
        "waiting time": [],
        "travel time": [],
        "detour ratio": []
    }

    # Retrieve TaxiReservation info using Traci
    # 택시 예약 정보(TaxiReservation) 조회
    for uid in traci.person.getTaxiReservations(0):
        pid = uid.persons[0]
        print_user["ID"].append(pid)
        print_user["pick-up"].append(uid.fromEdge)
        print_user["drop-off"].append(uid.toEdge)

        # state == 4 means the reservation is assigned to a taxi
        # state == 4: 해당 예약이 택시에 할당된 상태
        if uid.state == 4:
            print_user["waiting time"].append(user_wt[pid])
            print_user["travel time"].append(user_tt[pid])
            print_user["detour ratio"].append(user_dtr[pid])

    print("=== Requests ===")
    print(tabulate(print_user, headers="keys", tablefmt="fancy_grid", missingval="N/A"))

def print_veh_info(veh_users, veh_route):
    """
    Prints the assigned users and route information for each vehicle (taxi) in tabular form.
    :param veh_users: A dictionary of user lists assigned to each vehicle
    :param veh_route: A dictionary of stop (waypoint) lists for each vehicle

    최적화 결과로 각 차량(택시)에 할당된 사용자 목록과
    실제 주행 경로(route) 정보를 표로 출력한다.
    :param veh_users:  각 차량에 할당된 사용자 목록
    :param veh_route: 각 차량별 주요 스탑(정차 지점) 리스트
    :return:
    """
    print_veh = {
        "ID": [],
        "users": [],
        "path": [],
        "via": []
    }

    # Iterate over vehicles registered in opt.vehicles
    # 이미 opt.vehicles에 등록된 차량 정보를 순회
    for vid in opt.vehicles:
        print_veh["ID"].append(vid)
        print_veh["users"].append(veh_users[vid])
        print_veh["path"].append(veh_route[vid])

        # Current road ID of the vehicle
        # 현재 도로 정보
        if traci.vehicle.getRoadID(vid):
            print_veh["via"].append(traci.vehicle.getRoadID(vid))
        else:
            print_veh["via"].append("currently booking")

    print("=== Vehicles ===")
    print(tabulate(print_veh, headers="keys", tablefmt="fancy_grid", missingval="N/A"))

if 'SUMO_HOME' in os.environ:
    # Add SUMO tools to Python path
    # SUMO 툴 디렉토리를 path로 등록
    sys.path.append(os.path.join(os.environ['SUMO_HOME'], 'tools'))

    # Configure SUMO (GUI version) and the simulation configuration file
    # 시뮬레이션 실행 바이너리(여기서 GUI 버전) 및 설정
    sumoBinary = "/usr/bin/sumo-gui"
    sumoCmd = [sumoBinary, "-c", "./env/large_scale/osm.sumocfg"]

    # Start the SUMO simulation
    # SUMO 시뮬레이션 시작
    traci.start(sumoCmd)

    # Detour threshold parameter (maximum allowed detour ratio)
    # detour threshold 설정 (허용 가능 우회비율)
    dtr_thr = 3.0

    # Create an instance of the Optimization class
    # Optimization 인스턴스 생성
    opt = Optimization()

    # Dictionary to store vehicle route results
    # 차량 경로 결과를 저장할 딕셔너리
    result_vid_route = dict()

    # Run the simulation for 350 steps
    # 시뮬레이션 스텝 반복
    for step in range(350):
        traci.simulationStep()      # SUMO 시뮬레이션 한 스텝 진행

        # Print the current simulation time
        # 현재 시뮬레이션 시간을 출력
        print(f"Current simulation time: {int(traci.simulation.getTime())} seconds")

        # Add vehicles to the optimization model
        # 차량 정보를 opt 클래스에 업데이트
        for vid in traci.vehicle.getIDList():
            route = list(traci.vehicle.getRoute(vid))
            # If the vehicle only has one edge, store its initial position
            # 차량이 아직 한 개의 Edge만 가지고 있다면, 출발점 정보만 등록
            if len(route) == 1:
                opt.add_vehicles(vid, route)
            else:
                # If it already has a route, use the previously stored route info
                # 이미 경로가 있으면 result_vid_route에서 가져와 등록
                opt.add_vehicles(vid, result_vid_route[vid])

        # Add users to the optimization model
        # 사용자(승객) 정보를 opt 클래스에 업데이트
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

        # If there are no vehicles or no users to dispatch, skip optimization
        # 최적화 수행할 만한 차량 혹은 사용자가 없는 경우 스킵
        if len(opt.vehicles) == 0 or len(opt.users) == 0:
            continue
        else:
            # Perform optimization with the specified detour threshold
            # detour threshold 바탕으로 최적화 수행
            result_vid_route, result_vid_users, result_user_wt, result_user_tt, result_user_dtr = opt.opt(dtr_thr)

            # Print the results in tabular form
            # 결과를 출력 (표 형식)
            print_users_info(result_user_wt, result_user_tt, result_user_dtr)
            print_veh_info(result_vid_users, result_vid_route)

    # Close the simulation
    # 시뮬레이션 종료
    traci.close()