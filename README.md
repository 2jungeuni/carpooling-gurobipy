# carpooling-gurobipy

---
The experiment optimize real-time matching of ride-hailing vehicles to service requests using the Gurobi solver.

![video.gif](./result/video.gif)
---
## In English :uk:
## Local installation
### Download this project
```bash
git clone git@github.com:2jungeuni/carpooling-gurobipy.git
```
### Create a conda environment
```bash
cd ./carpooling-gurobipy
conda env create -f environment.yaml
conda activate flow
```
### Get the Gurobi license
It is recommended to follow the installation instructions provided in the [video](https://www.youtube.com/watch?v=OYuOKXPJ5PI).

### Install SUMO
It is recommended to follow the installation instructions provided in the official [SUMO documentation](https://sumo.dlr.de/docs/Installing/index.html).

## Experiments
- Each system request has a pick-up location and a drop-off location (```./env/large_scale/osm.person.xml```).
- A vehicle waits at the desired location and moves to the pick-up location as soon as passenger is matched (```./env/large_scale/osm.rou.xml```).
- Each passenger cannot travel directly from their pick-up location to their drop-off location when sharing the ride with other passengers. This inevitably results in detours, and our system is designed to reject matches when the detours are excessive.
  - The detour ratio is defined as the travel time of the detour divided by the travel time of a solo ride.
  - A match is rejected if the detour ratio exceeds 3.0.

---
## In Korea :kr:
## 설치 방법
### 프로젝트 다운로드
```bash
git clone git@github.com:2jungeuni/carpooling-gurobipy.git
```
### Conda 환경 생성
```bash
cd ./carpooling-gurobipy
conda env create -f environment.yaml
conda activate flow
```
### Gurobi 라이선스 받기
[영상](https://www.youtube.com/watch?v=OYuOKXPJ5PI)의 설치 지침을 따르는 것을 권장합니다.

### SUMO 설치
[공식 SUMO 문서](https://sumo.dlr.de/docs/Installing/index.html)을 따르는 것을 권장합니다.

### 세부 실험 내용
- 각 시스템 요청은 탑승 위치와 하차 위치를 포함합니다 (```./env/large_scale/osm.person.xml```).
- 차량은 지정된 위치에서 대기하며, 승객이 매칭되면 즉시 픽업 위치로 이동합니다 (```./env/large_scale/osm.rou.xml```).
- 승객은 다른 승객과 함께 탑승할 경우 탑승 위치에서 하차ㅇ 위치로 바로 이동할 수 없으며, 이는 필연적으로 우회 경로를 발생시킵니다. 본 시스템은 우회가 과도할 경우 매칭을 거부하도록 설계하였습니다.
  - 우회율(detour ratio)는 우회 경로의 주행 시간을 단독 주행 시 소요되는 시간으로 나눈 값으로 정의합니다.
  - 우회율 (detour ratio)이 3.0을 초과하면 매칭이 거부됩니다.