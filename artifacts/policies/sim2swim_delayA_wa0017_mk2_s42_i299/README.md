# sim2swim_delayA_wa0017_mk2_s42_i299

`fixplant_wa0017` 계보에 **행동 지연 DR(40~80 ms) + 관측 신선도 15%** 를 더해
재학습한 정책 (설계 A, 계약 유지). 근본원인 확정(2026-09-02 실기: dead time
80 ms + 포화 relay → 2 Hz limit cycle) 이후 학습 측 대응의 1차 후보다.

- 학습: `paper_delay_v1` (paper_ref_v1 + 지연 DR), seed 42, 2048 env, 300 iter,
  `w_a=0.017`. 스펙·결과: OceanRL_test `step_2_BROV/DELAY_TRAINING_PLAN.md`.
- **관측/행동 계약은 16-D v2 그대로** — 지연 DR 은 학습 시점 전용이라 배포
  artifact 는 기존과 같은 정적 사상이다. 배포측 변경 없음.
- IsaacLab gate: 주입 80 ms 에서 |ω| p90 = baseline 의 1/14, 포화 0%,
  지연 0 추종률 100.0%. 순항 Jacobian K_p 7.2 → 2.6 (80 ms 문턱 3.52 아래).
- **저지연에서도 안정** (0~80 ms 전 구간) — 28-D 설계 B 와 달리
  out-of-distribution 붕괴가 없다.
- SITL 검증: 통과 (DELAY_TRAINING_PLAN §5-11 — relay cycle 소멸, 포화 0%).
- **실기 검증: 통과 (2026-09-03 수조, session_20260903_summary.md 실험 2)** —
  포화 58→7%, v_e 평균 ≈0, 속도 +70%, 다리당 18.3→6.1 s.
- **2026-09-03 정본 배포 번들로 승격.** 대조군은 fixplant 번들 (아래 참조) —
  비교 주행 시 같은 자리 짝 주행 (runtime/a3_policy.sh).
