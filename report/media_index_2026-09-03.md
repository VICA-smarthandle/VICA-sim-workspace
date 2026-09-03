# media — 보고서에 바로 쓰는 영상·이미지

여기 있는 것만 인용한다. `oldones/` 는 조건이 맞지 않던 시절의 것이고
`oldones/README.md` 가 이유를 적어 뒀다.

## 1. planner × controller 비교 (`matrix/`)

플래너 하나당 영상 하나, 그 안에 컨트롤러 셋이 나란히 나온다.

| 코스 | NavFn | Smac Hybrid | Smac 2D | Smac Lattice |
|---|---|---|---|---|
| 코너링 | `matrix/corner_1.20_navfn.mp4` | `corner_1.20_hybrid.mp4` | `corner_1.20_smac2d.mp4` | `corner_1.20_lattice.mp4` |
| 동적 회피 | `matrix/avoid_1.20_navfn.mp4` | `avoid_1.20_hybrid.mp4` | `avoid_1.20_smac2d.mp4` | `avoid_1.20_lattice.mp4` |
| 유턴 | `matrix/uturn_1.20_navfn.mp4` | `uturn_1.20_hybrid.mp4` | `uturn_1.20_smac2d.mp4` | `uturn_1.20_lattice.mp4` |

`*_compare.*` 는 코스 전체가 보이는 넓은 화면, `*_closeup.*` 는 로봇을 크게 잡은
것, `*_storyboard.png` 는 한 장에 요약한 정지 이미지다.

판정은 `report/matrix_2026-09-03.md`, 동적 장애물 수치는
`report/obstacle_response_2026-09-03.md`.

## 2. 병원·오피스 주행

| 파일 | 시점 | 무엇이 보이나 |
|---|---|---|
| `hospital_trail.mp4` | trail | 로봇이 지나온 자리에서 따라간다. 벽 0% |
| `hospital_wide.mp4` | trail 14 mm | 1.8 m 뒤에서 광각. 병실이 넓게 |
| `office_lobby_trail.mp4` | trail | 로비를 가로질러 복도로 들어간다 |
| `office_lobby_wide.mp4` | follow 14 mm | 넓은 화각, 경로선 없음 |
| `office_lobby_chase.mp4` | chase | 어깨 너머로 진행 방향을 본다 |

`office_lobby_angles.png` 는 오피스 세 시점을 한 장에 비교한 것이다.

오피스는 **로비 방향**으로 주행한 것이다. 창문 쪽으로 찍힌 예전 영상은 지웠고,
`hospital_tour` 도 지웠다 — 프레임의 25% 가 벽 클로즈업이었다.

실내에서 `follow` 가 되는 곳은 트인 로비뿐이다. 병원 복도에서는 14 mm 로도
프레임의 23% 가 벽이었다. 그래서 병원은 두 편 다 `trail` 이고 화각과 거리만
다르다.

## 3. 로봇팔

| 파일 | 무엇 |
|---|---|
| `omx_arm_length_compare.png` | 같은 자리·같은 카메라, 팔 길이만 다름 |
| `omx_arm.mp4` | 설계 길이 0.30 / 0.28 |
| `omx_arm_stock.mp4` | 로보티스 기본 0.128 / 0.124 |

근거는 `report/arm_length_2026-09-03.md`.

## 4. 실내에서 찍을 때

`follow` 는 로봇을 담을 거리만큼 뒤로 물러난다. 복도는 그만큼 넓지 않아서
24 mm 는 프레임의 4분의 1, 18 mm 는 전부가 벽 클로즈업이 된다. 실내에서는

    --view trail --focal 20        # 벽에 절대 안 들어간다
    --view follow --focal 14       # 넓게 보고 싶을 때

두 가지만 쓴다. 자세한 이유는 `replay_render.py` 의 `--view` 설명에 있다.
