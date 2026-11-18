<p align="center">
  <img src="docs/_static/BAM_logo.png" alt="BAM logo" width="60%">
</p>

# BAM: Better Actuator Models

> **이 저장소는 [Rhoban/bam](https://github.com/Rhoban/bam) 의 포크입니다.**
> 아래 "이 포크에서 바꾼 점" 은 Kid_RL 휴머노이드(다리 MX-106(2.0)·XH540,
> 고관절 yaw MX-64(2.0), 팔·목 MX-28(2.0))를 mjlab / MuJoCo Warp 에서 학습시키려고
> 바꾼 내용입니다. 기준은 upstream 커밋 `4e060cf` 이고, 그 뒤 upstream 커밋은
> 아직 합치지 않았습니다. 원본 README 는 그 아래에 그대로 있습니다.

## 이 포크에서 바꾼 점

### 1. 새 서보 모델 추가 (Protocol 2.0 Dynamixel)

| 키 (`motor_name`) | 서보 | 클래스 (`bam/dynamixel/actuator.py`) |
|---|---|---|
| `mx28` | MX-28(2.0) | `MX28Actuator` |
| `mx64v2` | MX-64(2.0) | `MX64V2Actuator` |
| `mx106v2` | MX-106(2.0) | `MX106V2Actuator` |
| `xh540` | XH540-W270 | `XHActuator` |

- **이유:** upstream 의 `mx64`, `mx106` 은 Protocol 1.0 펌웨어용이고 오실로스코프로 잰
  `error_gain=0.158` 를 씁니다. 로봇에 들어간 서보는 2.0 펌웨어라 컨트롤 테이블과
  P 게인 환산이 다릅니다(XH540 과 같은 테이블: Goal Position 116, Present PWM 124,
  PWM Limit 36, 4096 counts/rev).
- **error_gain 계산:** `(4096 / 2π) / (KP_DIVISOR · PWM_LIMIT)` 식으로 구합니다.
  - `KP_DIVISOR` 는 데이터시트에 128 로 나와 있습니다. 하지만 실측해 보면 MX-106(2.0)
    은 약 147, MX-64(2.0) 는 141.32 였습니다(아래 `configure_kp_divisor.py`).
    XL330 에서도 같은 종류의 차이가 있었습니다.
  - MX-28(2.0) 과 XH540 은 아직 데이터시트 값 128 을 씁니다.
- **파라미터:** 실제 서보를 진자 테스트벤치로 기록해서 `bam.fit` 으로 맞춘 m1–m6
  파라미터를 `bam/params/{mx28,mx64v2,mx106v2,xh540}/` 에 넣었습니다.
- **등록:** 네 클래스를 `bam/actuators.py` 의 `actuators` 사전에 넣었습니다.
  - `BamActuatorCfg(motor_name=...)` 는 `params/<motor_name>/<model>.json` 을 바로
    읽으므로, mjlab 쪽은 코드 변경 없이 새 키를 쓸 수 있습니다.

### 2. 식별(identification) 도구

- **`bam/dynamixel/dynamixel.py` 에 `DynamixelXH540` 추가:** dynamixel_sdk 로 직접 만든
  Protocol 2.0 드라이버이고, MX-28/64/106(2.0) 에도 그대로 씁니다.
  - 읽기를 재시도하고 검증합니다.
  - 하드웨어 오류 표시(ALERT 비트)와 실제 통신 오류를 구분합니다. 그래서 입력 전압
    오류가 계속 켜져 있어도 통신이 막히지 않습니다.
  - 위치 rad↔raw 환산을 이 하드웨어의 Position Control Mode(raw 0..4095, unsigned)에
    맞게 고쳤습니다. 식은 ROBOTIS `dynamixel_hardware_interface` 에서 가져왔습니다.
- **`bam/dynamixel/configure_kp_divisor.py` (신규):** 오실로스코프 없이 펌웨어 Kp
  divisor 를 잽니다. 작은 삼각파 목표를 주고, 서보가 낸 Present PWM 을 읽어서
  `KP_DIVISOR = kp · error / (duty · PWM_LIMIT)` 를 최소제곱으로 맞춥니다.
- **`bam/dynamixel/record.py`:**
  - 위 서보들은 `DynamixelXH540` 으로 기록합니다.
  - 기록 시작 전에 `--logdir` 를 만듭니다. 원래는 마지막 저장 단계에서야 폴더가 없다는
    걸 알게 됐습니다.
  - 로그 키 `"arm-mass"` → `"arm_mass"`. 원래 이름은 `testbench.py` 가 읽는 키와 달라서
    팔 질량이 조용히 0 으로 들어갔습니다.
- **`bam/dynamixel/all_record.py`:** 새 서보의 kp 목록을 추가했습니다.
  - mx64v2/mx106v2 의 kp 는 이미 가진 v1 로그(kp 4, 8, 16, 32)와 같은 duty cycle 이
    나오도록 환산한 값입니다.
  - `sin_time_square` 궤적 뒤 대기를 3 → 10 s 로 늘렸습니다(서보 냉각).
- **`bam/fit.py`:**
  - `--eval` 이 `--output` 을 무시하고 `params.json` 을 읽던 것을 고쳤습니다.
  - `--eval` 이 학습에 쓴 로그까지 섞어 점수를 내던 것도 고쳤습니다. 이제
    `--validation_kp` 를 주면 떼어 둔 로그만 평가합니다.
  - kp 별로 점수를 따로 내는 `--eval_all_kp` 를 추가했습니다.
- **`bam/model.py` 탐색 범위:** 새 서보 피팅에서 최적값이 탐색 범위 끝에 걸려서 범위를
  넓혔습니다.
  - `load_friction_motor_quad` 상한 0.01 → 0.3
  - Stribeck `alpha` 범위 [1, 10] → [0.1, 60]
- **`pyproject.toml`:** `optuna<4.4.0` 으로 고정했습니다. 4.4.0 부터 CmaEsSampler 의
  `restart_strategy` (bipop/ipop) 가 조용히 무시됩니다.

### 3. mjlab 액추에이터 (`bam/mjlab.py`)

- **토크 제한 버그 수정 (커밋 `41aa0c2`):**
  - `set_to_motor()` 는 `<position>` 액추에이터의 `ctrlrange` (관절 각도, rad) 를 그대로
    남깁니다. 그런데 motor 액추에이터의 ctrl 은 토크(Nm)라서, BAM 토크가 관절 각도 범위로
    잘렸습니다. 예를 들어 hip roll 은 ±1.19 Nm, 무릎은 1.59 Nm 로 잘렸고, 원래 한계는
    6–8 Nm 입니다.
  - `ctrlrange` 를 토크 한계로 다시 설정하게 고쳤습니다.
  - 나중에 upstream 도 같은 문제를 따로 고쳤습니다.
- **`BamActuatorCfg.preserve_joint_friction` 추가:** 여기 적은 관절은 BAM 이 XML 의
  `damping`/`frictionloss` 를 0 으로 지우지 않고, `stiff_frictionloss` 도 적용하지
  않습니다.
  - 링키지의 종동 관절처럼 regex 에 걸리지만 BAM 이 마찰을 계산하지 않는 관절에 씁니다.
- **`BamActuatorCfg.max_current` 필드 추가:** 아직 BAM 액추에이터로 전달되지 않아서
  지금은 아무 효과가 없습니다. 전류 제한 자체는 `bam/actuator.py` 의
  `VoltageControlledActuator(max_current=...)` 에 있습니다.
- **문서:** `motor_name` docstring 에 지금 들어 있는 서보 목록을 적었습니다.

### 4. 그 밖의 파일

- `bam/scratch_add_slot_followers.py`: Kid_RL 의 크랭크-슬라이더 기구를 MJCF 에 넣으려고
  만든 임시 스크립트입니다. 경로가 로컬로 고정돼 있습니다.

Accurate models of servo actuators are essential for the simulation of robotic systems. It is particularly important while performing Reinforcement Learning (RL) on real robots, as the precision of the model impacts directly the transferability of the learned policy.

The friction model generally implemented in widely used simulators like MuJoCo or IsaacGym is the Coulomb-Viscous, which is too simplistic to accurately represent complex friction phenomena like Stribeck, load-dependence or quadratic effects (read [this article](https://arxiv.org/pdf/2410.08650v1) for more details).

**BAM** aims at making servo-actuator simulation more faithful by:

- proposing an identification pipeline to fit friction models from recorded trajectories,
- providing a set of extended friction models that capture complex friction phenomena,
- sharing a library of identified models for common servos:
  - Dynamixel MX-64
  - Dynamixel MX-106
  - Dynamixel XL-320
  - Dynamixel XL330-288-T
  - eRob80:50
  - eRob80:100
  - Feetech STS3215
- providing a simple API to use these models in MuJoCo CPU and MuJoCo Warp.

## 📖 Documentation

Please refer to the [documentation](https://bam.readthedocs.io/en/latest/) for more details on how to use BAM and the provided friction models.

## 📚 Citation

If you use BAM or the friction models provided in this repository for your scientific work, please cite the following publication:

```bibtex
@inproceedings{duclusaud2025extended,
  title={Extended Friction Models for the Physics Simulation of Servo Actuators},
  author={Duclusaud, Marc and Passault, Gr{\'e}goire and Padois, Vincent and Ly, Olivier},
  booktitle={2025 IEEE International Conference on Robotics and Automation (ICRA)},
  pages={12091--12097},
  year={2025},
  organization={IEEE}
}
```

If you also wish to cite the software directly, you may use:

```bibtex
@software{BAM,
  title = {{BAM: Better Actuator Models}},
  author = {Duclusaud, Marc and Passault, Grégoire},
  license = {Apache-2.0},
  url = {https://github.com/Rhoban/bam},
  version = {0.0.1},
  year = {2024}
}
```
