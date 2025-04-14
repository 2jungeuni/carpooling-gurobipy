## SUMO + Gurobi Taxi Dispatch Optimization
---
This project demonstrates how to use [SUMO (Simulation of Urban MObility)](https://www.eclipse.org/sumo/) alongside the [Gurobi](https://www.gurobi.com/) solver to perform vehicle-user matching and routing optimization.  
Users appear as taxi reservations in the SUMO simulation, and we solve a mixed-integer linear programming (MILP) problem to assign vehicles to users while respecting constraints such as **detour ratios**.

![video.gif](./result/video.gif)
---
### In English :uk:
#### Local installation
#### Download this project
```bash
git clone git@github.com:2jungeuni/carpooling-gurobipy.git
```
#### Create a conda environment
```bash
cd ./carpooling-gurobipy
conda env create -f environment.yaml
conda activate flow
```
#### Get the Gurobi license
It is recommended to follow the installation instructions provided in the [video](https://www.youtube.com/watch?v=OYuOKXPJ5PI).

#### Install SUMO
It is recommended to follow the installation instructions provided in the official [SUMO documentation](https://sumo.dlr.de/docs/Installing/index.html).

#### Experiments
1. **SUMO Simulation**
`main.py` starts SUMO (in GUI mode here) and reads the positions of vehicles and user reservations each time step.
```bash
python3 main.py
```
2. **Gurobi Optimization**
`opt.py` defines a MILP for matching vehicles to user requests (pick-up & drop-off points) and sets an allowed detour threshold.
If a user's detour ratio is too high, that user may be removed from the matching solution.
3. **Terminal Output**
After each optimization step, the script displays tabulated data regarding assigned users (waiting time, travel time, detour ratio) and the route each vehicle will take.

---
### In Korean :kr:
#### 설치 방법
#### 프로젝트 다운로드
```bash
git clone git@github.com:2jungeuni/carpooling-gurobipy.git
```
#### Conda 환경 생성
```bash
cd ./carpooling-gurobipy
conda env create -f environment.yaml
conda activate flow
```
#### Gurobi 라이선스 받기
[영상](https://www.youtube.com/watch?v=OYuOKXPJ5PI)의 설치 지침을 따르는 것을 권장합니다.

#### SUMO 설치
[공식 SUMO 문서](https://sumo.dlr.de/docs/Installing/index.html)을 따르는 것을 권장합니다.

#### 실험 방법
1. **SUMO 시뮬레이션**
`main.py`는 SUMO를 실행하고, 각 시뮬레이션 스텝마다차량과 사용자(예약) 정보를 읽어옵니다.
```bash
python3 main.py
```
2. **Gurobi 최적화**
`opt.py`에서는 차량과 사용자 요청(픽업, 드롭오프 지점)을 미챙하기 위한 MILP(혼합정수계획) 모델을 정의하며, 허용 가능한 우회 비율을 설정합니다.
만약 특정 사용자의 우회 비율이 지나치게 높으면 최적해에서 해당 사용자를 제외할 수 있습니다.
3. **터미널 출력**
각 최적화 단계를 거칠 때마다, 스크립트는 할당된 사용자들의 대기 시간, 이동 시간, 우회 비율 및 각 차량이 이동할 경롤르 표 형태로 터미널에 출력합니다.
