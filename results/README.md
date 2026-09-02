# results/

| 디렉터리 | 무엇을 잰 것인가 | 언제 |
| --- | --- | --- |
| `matrix/` | controller 3 x planner 4, 코너·회피·유턴 | 2026-09-02~ |

## 8월 결과를 지운 이유

`20kg` `94kg` `avoid_20kg` `avoid_20kg_mppi` `avoid_narrow`
`avoid_narrow_ring_bypass` 여섯 개를 지웠습니다. 전부 결함이 있는 시뮬에서 나온
값이고, 새 표와 같은 자리에 두면 섞입니다.

결함은 두 가지였고 둘 다 그 기간 내내 있었습니다.

  - `camera_link` 에 `<inertial>` 이 없었습니다. `merge_fixed_joints` 를 끄고
    수입하므로 그 링크는 자기 강체가 되는데 무게가 없어서, PhysX 가 재생마다
    "possibly invalid inertia tensor of {1.0, 1.0, 1.0} and a negative mass" 를
    찍었습니다. 관절체는 한 덩어리로 풀리므로 질량 행렬 전체가 같이 망가집니다.
    VICA.xacro 를 2026-08-04 까지 거슬러 확인했고, 그 시점에도 없었습니다.
  - 코스 바닥이 50 m 짜리 상자 하나였습니다. PhysX 가 그 상자와 65 mm 바퀴
    원통의 접촉을 자리에 따라 놓쳤습니다. 몸통 상자만 닿아서 로봇이 배를 깔고
    앉았습니다.

고친 뒤 측정: 열 개 지점 전부 z 0.190 (전에는 자리에 따라 0.142~0.190),
명령 2·4·6·8 rad/s 전부 r*omega 의 100 %, nav2 통과 4.41 m (전에는 0.00 m).

되살리려면:

    git checkout f44f5e4~1 -- results/20kg          # 필요한 것만

고친 커밋: f44f5e4
