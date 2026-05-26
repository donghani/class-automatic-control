# Quarter Car Active Suspension Simulation

> **2-DOF 쿼터 카 모델**에 능동 액추에이터를 추가하고,  
> **PID 제어기**와 **상태 피드백(State Feedback) 제어기**의 성능을 비교하는 MATLAB / Simulink 시뮬레이션입니다.

---

## 목차
1. [물리 모델](#1-물리-모델)
2. [운동 방정식 유도](#2-운동-방정식-유도)
3. [상태공간 표현](#3-상태공간-표현)
4. [제어 가능성 확인](#4-제어-가능성-확인)
5. [PID 제어기 설계](#5-pid-제어기-설계)
6. [상태 피드백 + 적분 제어기 설계](#6-상태-피드백--적분-제어기-설계)
7. [Simulink 모델 구조](#7-simulink-모델-구조)
8. [실행 방법](#8-실행-방법)
9. [결과 해석](#9-결과-해석)
10. [파라미터 변경 방법](#10-파라미터-변경-방법)

---

## 1. 물리 모델

### 쿼터 카(Quarter Car)란?

실제 자동차는 4개의 바퀴가 있지만, 대칭성을 가정하면 **한 바퀴의 수직 거동**만으로 전체 거동을 근사할 수 있습니다.  
이를 **쿼터 카(1/4 차량)** 모델이라고 합니다.

```
           [  차체  Mc  ]   ← 우리가 제어하고 싶은 높이 yc
                /\
               /  \
          ks, cs   F_act   ← 스프링·댐퍼 (서스펜션) + 액추에이터
               \  /
            [  휠  Mw  ]   ← 바퀴 높이 yw
                /\
               /  \
              kt, ct       ← 타이어 (스프링·댐퍼)
               \  /
         ▓▓▓▓▓▓▓▓▓▓▓▓▓    ← 노면 (yr, 외란)
```

### 사용된 파라미터

| 기호 | 의미 | 기본값 | 단위 |
|------|------|--------|------|
| `Mc` | 차체(스프링 상부) 질량 | 300 | kg |
| `Mw` | 휠(스프링 하부) 질량 | 50 | kg |
| `ks` | 서스펜션 스프링 강성 | 15,000 | N/m |
| `cs` | 서스펜션 댐퍼 계수 | 1,500 | N·s/m |
| `kt` | 타이어 스프링 강성 | 200,000 | N/m |
| `ct` | 타이어 댐퍼 계수 | 0 | N·s/m |
| `F_act` | 액추에이터 힘 (제어 입력) | — | N |
| `yc` | 차체 수직 변위 | — | m |
| `yw` | 휠 수직 변위 | — | m |
| `yr` | 노면 변위 (외란) | 0 | m |

---

## 2. 운동 방정식 유도

뉴턴의 제2법칙(F = ma)을 각 질량에 적용합니다.

### 차체 (Mc)에 작용하는 힘

차체에는 세 가지 힘이 작용합니다:
- 서스펜션 스프링 힘: `ks(yc - yw)` — 아래 방향 (복원력)
- 서스펜션 댐퍼 힘: `cs(ẏc - ẏw)` — 아래 방향 (감쇠력)
- 액추에이터 힘: `F_act` — 위 방향 (제어 입력)

$$M_c \ddot{y}_c = -k_s(y_c - y_w) - c_s(\dot{y}_c - \dot{y}_w) + F_{act}$$

### 휠 (Mw)에 작용하는 힘

휠에는 네 가지 힘이 작용합니다:
- 서스펜션 스프링 반력: `ks(yc - yw)` — 위 방향
- 서스펜션 댐퍼 반력: `cs(ẏc - ẏw)` — 위 방향
- 타이어 스프링 힘: `kt(yw - yr)` — 아래 방향
- 액추에이터 반력: `F_act` — 아래 방향 (작용-반작용)

$$M_w \ddot{y}_w = k_s(y_c - y_w) + c_s(\dot{y}_c - \dot{y}_w) - k_t(y_w - y_r) - c_t \dot{y}_w - F_{act}$$

> 노면 외란 yr = 0 으로 설정 (평탄 도로 가정)

---

## 3. 상태공간 표현

### 상태 변수 정의

4개의 상태 변수를 정의합니다:

$$\mathbf{x} = \begin{bmatrix} x_1 \\ x_2 \\ x_3 \\ x_4 \end{bmatrix} = \begin{bmatrix} y_c \\ \dot{y}_c \\ y_w \\ \dot{y}_w \end{bmatrix}$$

### 상태 방정식 도출

운동 방정식을 1차 미분 방정식으로 변환합니다:

$$\dot{\mathbf{x}} = A\mathbf{x} + B u$$

$$A = \begin{bmatrix} 0 & 1 & 0 & 0 \\ -\frac{k_s}{M_c} & -\frac{c_s}{M_c} & \frac{k_s}{M_c} & \frac{c_s}{M_c} \\ 0 & 0 & 0 & 1 \\ \frac{k_s}{M_w} & \frac{c_s}{M_w} & -\frac{k_s+k_t}{M_w} & -\frac{c_s+c_t}{M_w} \end{bmatrix}$$

$$B = \begin{bmatrix} 0 \\ \frac{1}{M_c} \\ 0 \\ -\frac{1}{M_w} \end{bmatrix}, \quad C = \begin{bmatrix} 1 & 0 & 0 & 0 \end{bmatrix}, \quad D = 0$$

수치 대입 (Mc=300, Mw=50, ks=15000, cs=1500, kt=200000):

$$A = \begin{bmatrix} 0 & 1 & 0 & 0 \\ -50 & -5 & 50 & 5 \\ 0 & 0 & 0 & 1 \\ 300 & 30 & -4300 & -30 \end{bmatrix}, \quad B = \begin{bmatrix} 0 \\ 0.00\overline{3} \\ 0 \\ -0.02 \end{bmatrix}$$

### MATLAB 코드

```matlab
%% 상태공간 행렬 구성
A = [0,       1,        0,             0;
    -ks/Mc,  -cs/Mc,   ks/Mc,         cs/Mc;
     0,       0,        0,             1;
     ks/Mw,   cs/Mw,  -(ks+kt)/Mw,  -(cs+ct)/Mw];

B_act = [0;  1/Mc;  0;  -1/Mw];  % 액추에이터 입력
C_out = [1,  0,  0,  0];          % 출력: yc
D_out = 0;

sys = ss(A, B_act, C_out, D_out); % MATLAB ss 객체 생성
```

---

## 4. 제어 가능성 확인

### 제어 가능성(Controllability)이란?

**제어 가능한 시스템** = 유한한 시간 내에 입력 u로 임의의 상태 x에 도달 가능한 시스템

제어 가능성 행렬(Controllability Matrix)의 랭크가 시스템 차수 n과 같으면 제어 가능합니다:

$$\text{rank}(\mathcal{C}) = \text{rank}\begin{bmatrix} B & AB & A^2B & A^3B \end{bmatrix} = n = 4$$

### MATLAB 코드

```matlab
%% 제어 가능성 확인
if rank(ctrb(A, B_act)) == 4
    disp('제어 가능 (Controllable) ✓');
else
    error('제어 불가능 — 파라미터를 확인하세요.');
end
```

---

## 5. PID 제어기 설계

### PID 제어기 구조

$$F_{act}(t) = K_p \, e(t) + K_i \int_0^t e(\tau)\,d\tau + K_d \, \frac{de(t)}{dt}$$

여기서 오차는 $e(t) = y_{c,ref} - y_c(t)$

**주파수 영역** 전달함수:

$$C_{PID}(s) = K_p + \frac{K_i}{s} + \frac{K_d \, s}{1 + T_f s}$$

> $T_f$ : 미분 필터 시상수 (고주파 노이즈 억제)

### 제어기 설계 목표

| 지표 | 목표 |
|------|------|
| 오버슈트 (Overshoot) | < 5% |
| 상승 시간 (Rise Time) | < 0.5 s |

### 폐루프 구성

```
         e(t)             F_act
r ──►[Σ]──►[PID]──────────────►[Plant]──► yc
     - ↑                               │
       └───────────────────────────────┘
```

폐루프 전달함수:

$$T(s) = \frac{C_{PID}(s) \cdot G(s)}{1 + C_{PID}(s) \cdot G(s)}$$

### 설계 방법: `pidtune` 대역폭 탐색

`pidtune`은 원하는 대역폭(bandwidth)을 입력받아 PID 게인을 자동 계산합니다.  
여러 대역폭을 시도하여 오버슈트와 상승 시간 조건을 동시에 만족하는 최적값을 선택합니다.

### MATLAB 코드

```matlab
%% PID 설계: 대역폭 탐색
best_pid   = [];
best_score = inf;

for bw = [8, 10, 12, 15, 18, 20, 25, 30]   % 시도할 대역폭 [rad/s]
    C_try = pidtune(sys, 'PID', bw);         % PID 자동 튜닝
    T_try = feedback(series(C_try, sys), 1); % 단위 음성 피드백 폐루프
    si    = stepinfo(T_try);                 % 계단 응답 지표 계산

    % 목표 달성 여부 확인
    if si.Overshoot < 5 && si.RiseTime < 0.5
        score = si.Overshoot + 10 * max(0, si.RiseTime - 0.5);
        if score < best_score
            best_score = score;
            best_pid   = C_try;   % 최적 PID 저장
        end
    end
end

C_pid = best_pid;
Kp = C_pid.Kp;   % 비례 게인
Ki = C_pid.Ki;   % 적분 게인
Kd = C_pid.Kd;   % 미분 게인
```

### 설계 결과

| 게인 | 값 |
|------|----|
| `Kp` | 12,210.1 |
| `Ki` | 50,923.0 |
| `Kd` | 731.9 |
| 오버슈트 | **0.00 %** ✅ |
| 상승 시간 | **0.302 s** ✅ |

---

## 6. 상태 피드백 + 적분 제어기 설계

### 상태 피드백이란?

PID는 출력 `yc`만 보고 제어하지만, 상태 피드백은 **모든 상태 변수** (위치, 속도 2개씩)를 동시에 이용합니다.

$$F_{act} = -K \mathbf{x} = -\begin{bmatrix} k_1 & k_2 & k_3 & k_4 \end{bmatrix} \begin{bmatrix} y_c \\ \dot{y}_c \\ y_w \\ \dot{y}_w \end{bmatrix}$$

### 적분 상태 추가 (Reference Tracking)

순수 상태 피드백은 정상 상태 오차(steady-state error)가 발생할 수 있습니다.  
**적분 제어**를 추가하여 오차를 완전히 제거합니다.

적분 상태 정의: $x_{int} = \int_0^t (r - y_c) \, d\tau$

**확장 상태 벡터**:

$$\mathbf{z} = \begin{bmatrix} \mathbf{x} \\ x_{int} \end{bmatrix} = \begin{bmatrix} y_c \\ \dot{y}_c \\ y_w \\ \dot{y}_w \\ \int(r - y_c) \end{bmatrix}$$

**확장 시스템**:

$$\dot{\mathbf{z}} = A_{aug} \mathbf{z} + B_{aug} u + \begin{bmatrix} \mathbf{0} \\ 1 \end{bmatrix} r$$

$$A_{aug} = \begin{bmatrix} A & \mathbf{0} \\ -C & 0 \end{bmatrix}_{5 \times 5}, \quad B_{aug} = \begin{bmatrix} B \\ 0 \end{bmatrix}_{5 \times 1}$$

**확장 제어 법칙**:

$$F_{act} = -K_{aug} \mathbf{z} = -\begin{bmatrix} K_{sf} & K_{i,sf} \end{bmatrix} \begin{bmatrix} \mathbf{x} \\ x_{int} \end{bmatrix}$$

### 극점 배치(Pole Placement)

폐루프 극점의 위치가 시스템의 응답 특성을 결정합니다.

- **감쇠비** $\zeta = 0.8$ 선택 → 오버슈트 ≈ 1.5%
- **고유 진동수** $\omega_n = 15$ rad/s 선택 → 상승 시간 ≈ 0.2 s

**지배 극점 (주요 2개)**:

$$s_{1,2} = -\zeta \omega_n \pm j\omega_n\sqrt{1-\zeta^2} = -12 \pm 9j$$

**보조 극점 (빠른 3개)**:

$$s_3 = -60, \quad s_4 = -75, \quad s_5 = -45 \quad \text{(적분 극점)}$$

> 보조 극점은 지배 극점보다 **3~5배 더 왼쪽**에 배치하여 응답에 영향 최소화

### MATLAB 코드

```matlab
%% 확장 시스템 구성
A_aug = [A,     zeros(4,1);
        -C_out,  0        ];   % 5×5
B_aug = [B_act; 0];            % 5×1

%% 제어 가능성 확인 (확장 시스템)
assert(rank(ctrb(A_aug, B_aug)) == 5, '확장 시스템 제어 불가능');

%% 목표 극점 설정
zeta = 0.8;   wn = 15;
p1 = -zeta*wn + 1j*wn*sqrt(1-zeta^2);  % -12 + 9j
p2 = conj(p1);                           % -12 - 9j
p3 = -4*wn;   % -60  (빠른 극점)
p4 = -5*wn;   % -75  (빠른 극점)
p5 = -3*wn;   % -45  (적분 극점)

%% 극점 배치로 K 계산 (MATLAB place 함수)
K_full = place(A_aug, B_aug, [p1, p2, p3, p4, p5]);
K_sf   = K_full(1:4);   % 상태 피드백 게인
Ki_sf  = K_full(5);     % 적분 게인
```

### 폐루프 시스템 구성

```matlab
%% 폐루프 확장 시스템
A_cl = A_aug - B_aug * K_full;  % 5×5 폐루프 A
B_cl = [zeros(4,1); 1];         % 기준 입력 r이 적분기로 들어옴
C_cl = [C_out, 0];              % 출력: yc (첫 번째 상태)

sys_sf_cl = ss(A_cl, B_cl, C_cl, 0);
```

### 설계 결과

| 게인 | 값 |
|------|----|
| `K_sf` | [527953.1, 34905.7, −450507.8, −2632.4] |
| `Ki_sf` | −3,417,187.5 |
| 오버슈트 | **1.22 %** ✅ |
| 상승 시간 | **0.190 s** ✅ |

---

## 7. Simulink 모델 구조

`build_qc_model.m`이 자동으로 생성하는 Simulink 모델의 구조입니다.

```
┌─────────────┐   ┌──────────────────────┐   ┌───────┐
│  Step_Ref   │──►│  PID_CL (State Space) │──►│       │
│ (0→0.05 m)  │   │  폐루프 PID 시스템   │   │  Mux  │──►[Scope]
│             │   └──────────────────────┘   │       │
│             │   ┌──────────────────────┐   │(2→1)  │
│             │──►│  SF_CL  (State Space) │──►│       │
└─────────────┘   │  폐루프 SF  시스템   │   └───────┘
                  └──────────────────────┘

┌─────────────┐   ┌─────────────┐
│   Mc_val    │   │   Mw_val    │   ← 더블클릭으로 질량 변경
│  (상수 블록) │   │  (상수 블록) │
└─────────────┘   └─────────────┘
   InitFcn: 시뮬레이션 시작 전 자동으로 컨트롤러 재설계
```

> **두 컨트롤러를 독립적인 폐루프 상태공간 블록으로 구현**하여  
> 동일한 Step 입력에 대한 응답을 동시에 비교합니다.

---

## 8. 실행 방법

### 환경 요구사항
- MATLAB R2019b 이상
- Control System Toolbox
- Simulink

### Step 1 — 저장소 다운로드 및 경로 이동

```matlab
cd('quarter_car_simulation')
```

### Step 2 — 컨트롤러 설계 및 비교 플롯 생성

```matlab
quarter_car_init
```

이 스크립트가 수행하는 작업:
1. 시스템 파라미터 설정 (Mc, Mw, ks, cs, kt, ct)
2. 상태공간 모델 구성 및 제어 가능성 검증
3. PID 게인 자동 튜닝 (pidtune + 대역폭 탐색)
4. 상태 피드백 게인 계산 (pole placement)
5. 두 제어기의 계단 응답 비교 플롯 출력
6. Workspace 변수 저장 (Simulink 모델에서 사용)

### Step 3 — Simulink 모델 생성 및 열기

```matlab
build_qc_model
```

### Step 4 — 시뮬레이션 실행 및 결과 플롯

```matlab
simOut = sim('quarter_car_active');
plot_qc_results(simOut)
```

---

## 9. 결과 해석

### 계단 응답 비교 (Step Response)

`quarter_car_init` 실행 시 자동으로 생성되는 그래프:

```
y_c [m]
 0.053 ┤
       │    PID (파란 실선)
 0.050 ┤─── ─── ─── ─── ─── ─── ─── ─── (Reference)
       │  ╭────────────────────────────
       │  │ SF (빨간 점선) : 빠른 응답
       │  │
 0.025 ┤  │ PID : 천천히 부드럽게 수렴
       │  │
 0.000 ┤──┤────────────────────────────► t [s]
       0  0.5   1.0   1.5   2.0   5.0
```

### 최종 수치 비교

| 지표 | PID | State Feedback | 목표 |
|------|-----|----------------|------|
| **오버슈트** | 0.00 % | 1.22 % | < 5 % |
| **상승 시간** | 0.303 s | 0.190 s | < 0.5 s |
| **정착 시간** | 1.640 s | 0.318 s | — |
| **최종 변위** | 0.05000 m | 0.05000 m | 0.05000 m |

### 두 제어기 특성 비교

| 특성 | PID | State Feedback |
|------|-----|----------------|
| **정보 활용** | yc 오차만 사용 | 모든 상태 (yc, ẏc, yw, ẏw) 사용 |
| **설계 복잡도** | 낮음 (3개 파라미터) | 높음 (5개 파라미터) |
| **응답 속도** | 느림 | 빠름 |
| **오버슈트** | 없음 | 매우 작음 |
| **정착 시간** | 길음 | 매우 짧음 |
| **상태 측정** | 불필요 | 필요 (또는 옵저버 추가) |
| **강건성** | 상대적으로 강건 | 모델 정확도에 민감 |

---

## 10. 파라미터 변경 방법

### 질량 변경 (Simulink GUI에서)

1. Simulink 모델 열기: `open_system('quarter_car_active')`
2. `Mc_val` 블록 더블클릭 → 값 수정 (예: 350)
3. `Mw_val` 블록 더블클릭 → 값 수정 (예: 45)
4. **Run** 버튼 클릭 (Ctrl+T)
5. `InitFcn` 콜백(`quarter_car_recalc_callback.m`)이 자동으로 컨트롤러를 재설계

### MATLAB 스크립트에서 직접 변경

```matlab
% 파라미터 수정
Mc = 350;    % 차체 질량 변경
Mw = 45;     % 휠 질량 변경

% 재설계 (quarter_car_init.m의 파라미터 섹션 수정 후 실행)
quarter_car_init
build_qc_model
simOut = sim('quarter_car_active');
plot_qc_results(simOut)
```

### 서스펜션 파라미터 변경

`quarter_car_init.m` 상단의 파라미터 섹션을 수정합니다:

```matlab
%% 1. PHYSICAL PARAMETERS (여기를 수정하세요)
Mc  = 300;      % 차체 질량 [kg]
Mw  = 50;       % 휠 질량 [kg]
ks  = 15000;    % 서스펜션 스프링 [N/m]
cs  = 1500;     % 서스펜션 댐퍼 [N·s/m]
kt  = 200000;   % 타이어 스프링 [N/m]
ct  = 0;        % 타이어 댐퍼 [N·s/m]
step_amp = 0.05; % 계단 입력 크기 [m]
```

---

## 파일 구조

```
quarter_car_simulation/
├── README.md                      ← 이 파일
├── quarter_car_init.m             ← 메인 설계 스크립트 (여기서 시작)
├── build_qc_model.m               ← Simulink 모델 자동 빌드
├── quarter_car_recalc_callback.m  ← 파라미터 변경 시 자동 재설계 콜백
├── quarter_car_plant.m            ← 플랜트 동역학 함수 (참고용)
├── plot_qc_results.m              ← 시뮬레이션 결과 비교 플롯
└── quarter_car_active.slx         ← 생성된 Simulink 모델
```

---

## 참고

- [Control System Toolbox — pidtune](https://www.mathworks.com/help/control/ref/dynamicsystem.pidtune.html)
- [Control System Toolbox — place](https://www.mathworks.com/help/control/ref/place.html)
- [Simulink — State Space Block](https://www.mathworks.com/help/simulink/slref/statespace.html)
- Rajamani, R., *Vehicle Dynamics and Control*, Springer, 2012 — Chapter 12: Active Suspension
