# db_structures

table Node
start_idx/ end_idx/ id / mission_state / position (path_planning에 사용하진 않음)

table Path
idx / id / x / y / yaw
idx / id / x / y / yaw
idx / id / x / y / yaw
idx / id / x / y / yaw
...

다른 노드에 진입 --> mission_state check
A1 Start_point: position 
	 index: 0
A2 avoidance_start:     position:
      x: 18.148517608642578
      y: 43.508506774902344
      z: 0.0
     index:400
B1 avoidance_end:     position:
      x: 18.148517608642578
      y: 43.508506774902344
      z: 0.0
      index:457
 B2 parking_start :     position:
      x: 37.937705993652344
      y: 81.07661437988281
      z: 0.0
      index:1270
  C1 parking_end:     position:
      x: 18.148517608642578
      y: 43.508506774902344
      z: 0.0
      index:1316
C2 End_point: positon :?
	index -1

path 분할을 idx 기준으로 해보자
 Node 좌표 이상하다
1. path  분할 idx changing
2. logic 추가 , parameter pub(근데 그냥 코드를 하나에 다 합치면 되기도 함)
3. node 추가
----------------------------------------------------------------------

2024_acca_statemachine_초안
왜 안쓰게 되었는가?
1.큰그림을 그리는데 배경지식이 부족

2.코드 구조가 좋지 않음


