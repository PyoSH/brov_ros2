# sim2swim_fixplant_wa0017_mk2_s42_i299

`sim2swim_paperfix_wa0017_mk2_s42_i299`을 **수정된 plant 위에서 재학습**한 artifact.
보상/관측/행동 계약은 전부 동일하다 (`paper_ref_v1`, `w_a = 0.017`, seed 42, 2048 env, 300 iter).

## plant에서 무엇이 바뀌었나

`OceanRL_test/step_2_BROV/robots/dynamics/fossen.py` 세 곳:

1. **Coriolis 힘 부호** — `-cross(v_ang, M_A v_lin)`이 인자 순서가 뒤집혀 있었다.
   부호가 반대라 차량이 선회 시 바깥으로 밀리는 대신 **안쪽으로 당겨졌다.**
   skew-symmetry 검사값 `ν·(C_A ν)`이 30.07 → 1.0e-06으로 떨어졌다.
2. **가속도 저역필터 제거** — `_ACC_ALPHA = 0.3`으로 `ν̇`을 필터링해 added mass를 explicit
   외력으로 넣던 것을, `M_total = M_RB + M_A`를 직접 푸는 implicit 방식으로 교체했다.
   10 Hz 유효질량이 18.74 → 31.99 kg (참값 33.31).
3. `rigid_mass` / `rigid_inertia`를 생성자 인자로 받아 `M_total`에 반영.

**`deploy/vendor/brov2_heavy.yaml`은 바뀌지 않았다.** 감쇠·added mass 계수 등 파라미터는
그대로고, 바뀐 것은 그 계수를 쓰는 적분 방식이다. 따라서 metadata의
`vehicle_model_sha256`은 이전 artifact와 **동일한 `8bb397f4…`**이며, 이 해시로는 구/신
plant를 구분할 수 없다. 구분은 `checkpoint_sha256`(`b3245eab…`)과
`training_manifest_sha256`(`3ef5afc1…`)으로 한다.

## 재학습이 결과를 바꿨나 — 학습 곡선 기준으로는 아니다

동일 seed(42) 동일 iteration `Train/mean_reward`:

| iter | 신 plant | 구 plant | 차이 |
|---:|---:|---:|---:|
| 50 | 112.14 | 112.26 | −0.12 |
| 100 | 118.75 | 118.22 | +0.53 |
| 150 | 122.00 | 122.11 | −0.11 |
| 200 | 123.65 | 124.04 | −0.40 |
| 250 | 124.10 | 124.56 | −0.46 |

전 구간 차이 0.5 이내, 부호도 일관되지 않는다. `mean_episode_length`는 양쪽 모두 전 구간
124.0(조기 종료 없음). 이 mission의 정상상태 `|ω|`가 0.004 rad/s 수준이라 Coriolis 항이
유의미해지는 각속도 영역을 거의 쓰지 않기 때문으로 본다. 물리 버그 수정은 그 자체로 정당하지만
**sim2real 실패의 원인은 아니었다** — 원인은 `w_a` 단독이다.

## 검증 — velocity_hold [0.5, 0, 0], 후반 50% 정상상태

같은 plant 위에서 `w_a`만 바꾼 대조군과 함께 측정했다.

| 항목 | **이 artifact (w_a=0.017)** | w_a=0.3 (논문값) |
|---|---|---|
| surge 추종률 | **100.0%** (0.5001 ± 0.0002) | 16.4% (0.082 ± 0.061) |
| sway | −0.0005 | −0.174 |
| heave | +0.0011 | −0.0049 |
| surge action | 0.495 | 0.025 |
| `z_v` 최종 | [0.187, −0.105, 0.038] | **[−5.0(clamp 포화), −2.84, −0.09]** |
| roll / pitch / yaw | −0.49 / +0.07 / +0.70° (std ≤0.01) | +1.71±0.82 / +0.04 / −0.55° |
| force clamp 발생 | 0.0% | 0.0% |

읽을 점:

- surge action 0.495 → `F = 85 × 0.495 = 42.1 N`이고 이는 sim의 `drag(0.5)`와 같다. 즉
  `A = drag(V_d)/K_surge`의 예측치 0.486을 2% 이내로 재현한다. 수조 실측 41.3 N과 1.8% 차이.
- **양쪽 다 force clamp 0%다.** 16.4%는 추력 부족이 아니라 보상이 시킨 결과라는 것이
  같은 plant 위 직접 대조로 확인됐다. `w_a=0.3`은 적분기가 −5.0 clamp까지 밀렸는데도
  action을 0.025밖에 쓰지 않았다.
- `w_a=0.3`의 sway 편류 −0.174 m/s는 이 artifact에서 −0.0005로 사라진다. sway는
  `linear_damping = 0`이라 작은 action이 큰 속도로 증폭돼 보였던 것으로, 배선 문제가 아니라
  학습되지 않은 축의 잔여물이다.

## 아직 안 한 것

- **sim2sim 미실행.** Gazebo SITL(수정된 plant) 검증 이력 없음.
- **Fig.4 3-mission 미실행.** 위 표는 velocity_hold 단일 조건이다.
- 실기 시험 이력 없음.
- 속도 feedback은 여전히 ArduSub EKF3다. A50 DVL 직결 경로(layer 1)는 기록만 되고
  feedback으로 승격되지 않았다. 2026-08-28 수조 실측에서 EKF가 DVL 대비 12.9% 과소 보고,
  heave 축은 부호 반대였다.
- `w_a = 0.017`은 추종률 90% 목표를 역산한 값이라 그 자체로는 임의적이다.
  동적범위 균형점은 `w_a ≈ 0.115`(예측 49%).

## 출처

- checkpoint: `step_2_BROV/logs/fixplant_wa0017_s42/model_299.pt`
- 학습: `train.py --profile paper_ref_v1 --rew_w_action 0.017 --seed 42 --num_envs 2048 --max_iterations 300`
- gate: `test_policy.py --hold_velocity 0.5 0 0` → `_fp_0017.json`
