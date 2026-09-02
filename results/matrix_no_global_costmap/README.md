# 전역 costmap 을 맞추기 전 결과 — 새 표와 섞지 마십시오

2026-09-03 05:20 이전. 이 시점의 시뮬 설정에는 `global_costmap:` 절이 아예
없었고, 그래서 전역 플래너가 nav2 기본값으로 — 이 로봇의 육각 footprint 가
아니라 기본 반지름의 원으로 — 계획했다.

실물(`vica_ros2_ws/src/vica_nav2/config/nav2_params.yaml` 1279행)에는 그 절이
있고 지역과 같은 footprint 를 쓴다. 실물 쪽 주석은 그것을 이렇게 적어 뒀다:

> 두 costmap 이 다르면 planner 가 통과 가능하다고 본 경로를 controller 가
> 거부한다. 두 costmap 은 반드시 같은 footprint 를 써야 한다
> (test_local_and_global_costmap_use_the_same_footprint).

맞춘 뒤 같은 셀(DWB + NavFn, 코너 1.20 m)이 4.46 m 에서 1.33 m 로 떨어지고
Spin 복구에 111초를 쓴다. 결론이 바뀔 수 있는 크기라 전체를 다시 돌린다.

여기 값은 "전역 플래너가 로봇을 실제보다 작게 알 때 무엇이 되는가"의 기록으로만
쓸 수 있다.
