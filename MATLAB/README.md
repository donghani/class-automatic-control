# 🛠️ MATLAB 자동제어 실습 및 강의 자료실 (MATLAB Control Systems Lab)

이 디렉토리는 **자동제어공학** 수업의 실습 코드와 이를 직관적으로 이해할 수 있게 돕는 고품질 강의 자료(마크다운 문서)들을 모아놓은 공간입니다. 

자동제어의 양대 산맥인 **고전 제어(전달함수 기반)**와 **현대 제어(상태 공간 기반)**를 직접 매트랩으로 코딩해보고 시뮬레이션 결과를 그래프로 보며 제어공학의 정수를 체득할 수 있도록 체계적으로 구성되어 있습니다.

---

## 📂 폴더 구조 및 구성 요소 (Directory Structure)

```text
MATLAB/
├── 📈 transfer-function/      # [고전 제어] 전달함수 기반 제어 기법
│   ├── control_demo_1_pid.m         - PID 제어기 설계 및 비교 실습
│   ├── control_demo_2_analysis.m    - 시간/주파수 영역 안정성 해석 (Root Locus, Bode, Nyquist)
│   ├── control_demo_3_compensator.m - Lead/Lag 보상기 설계 실습
│   ├── control_demo_4_noise_filter.m - 미분기 노이즈 문제 및 저역통과필터(LPF) 결합
│   ├── root_locus_k.m               - 특정 시스템 파라미터 변화에 따른 근궤적 해석
│   ├── lecture_notes.md             - 고전 제어 이론 상세 강의 노트
│   ├── lecture_slides.md            - 수업 진행용 프리젠테이션 슬라이드 초안
│   ├── noise_and_derivative_filter.md - 노이즈 차단 필터 설계 심화 자료
│   ├── root_locus_k_parameter.md    - 파라미터 k에 대한 근궤적 수작업 작도법
│   ├── walkthrough.md               - 고전 제어 실습 가이드 및 그래프 분석
│   └── *.png                        - 각 단계별 제어 성능 비교 그래프 리소스
│
└── 🎛️ state-space/            # [현대 제어] 상태 공간 기반 제어 기법
    ├── control_demo_5_statespace.m  - 극점 배치, LQR, Observer, LQG 제어 통합 실습
    ├── lqr_5sec_design.m            - 5초 정착 시간 만족을 위한 LQR 최적 가중치 자동 튜닝
    ├── create_simulink_models.m     - 시뮬링크 연동용 블록 모델 (.slx) 자동 생성 스크립트
    ├── state_feedback_with_integral_ex.m - 상태 피드백 및 적분 제어 시뮬링크 연동 실습 코드
    ├── pole_placement.md            - 상태 피드백 제어와 스케일러 N̄ 수학적 유도
    ├── state_space_lecture.md       - 4단계 현대 제어 기법 빌드업 강의 노트
    ├── lqr_tutorial.md (.html)      - LQR 튜닝법 및 정상상태 오차 보정 상세 가이드
    ├── state_feedback_integral_control.md - 적분 제어 및 시뮬링크 실습 가이드 (수식 & 블록선도)
    ├── modern_control_design_guide.md - MATLAB 현대 제어 실무 설계 가이드
    └── *.png                        - LQR, Observer, LQG, Integral 제어 시스템 블록선도 및 시뮬레이션 결과
```
---

## 📈 1. 전달함수 기반 고전 제어 (`transfer-function/`)

전달함수(Transfer Function) 모델을 바탕으로 시스템의 주파수 및 시간 영역 특성을 분석하고, 원하는 응답 사양을 만족하기 위한 제어기를 설계합니다.

### 💻 실습 스크립트 (MATLAB Scripts)
* **[control_demo_1_pid.m](./transfer-function/control_demo_1_pid.m):** P, PI, PID 제어기 각각의 비례·적분·미분 게인이 과도 응답(오버슈트, 상승 시간, 정착 시간)과 정상상태 오차에 미치는 영향을 비교 분석합니다.
* **[control_demo_2_analysis.m](./transfer-function/control_demo_2_analysis.m):** 시스템의 폴(Poles)/제로(Zeros) 배치, 스텝 응답, 근궤적(Root Locus), 보드선도(Bode Plot), 나이퀴스트 선도(Nyquist Plot)를 그려 시스템의 주파수 영역 안정성(위상 여유, 이득 여유)을 평가합니다.
* **[control_demo_3_compensator.m](./transfer-function/control_demo_3_compensator.m):** 위상 진상 보상기(Lead Compensator, 속도 개선)와 위상 지상 보상기(Lag Compensator, 오차 개선)를 주파수 여유 설계법으로 설계합니다.
* **[control_demo_4_noise_filter.m](./transfer-function/control_demo_4_noise_filter.m):** 이상적인 미분기가 현실에서 노이즈를 어떻게 뻥튀기하는지 확인하고, 저역통과필터(LPF)가 결합된 실용적 미분기($\frac{s}{\tau s + 1}$) 설계법을 배웁니다.
* **[root_locus_k.m](./transfer-function/root_locus_k.m):** 제어 게인 $K$가 아닌 시스템 파라미터 $k$의 변화에 대한 근궤적을 수학적으로 등가 변환하여 분석합니다.

### 📚 강의 및 설명 문서 (Lecture Documents)
* **[lecture_notes.md](./transfer-function/lecture_notes.md):** 교수자용 종합 이론 강의 파일. PID 원리, 주파수 안정도 판별법, 보상기 설계 공식 수작업 유도 과정이 수록되어 있습니다.
* **[walkthrough.md](./transfer-function/walkthrough.md):** 학생들을 위한 단계별 실습 핸드아웃 가이드북.
* **[noise_and_derivative_filter.md](./transfer-function/noise_and_derivative_filter.md):** 미분 필터의 차단 주파수와 응답성 딜레마를 심도 깊게 다룬 심화 자료.
* **[root_locus_k_parameter.md](./transfer-function/root_locus_k_parameter.md):** 피드백 루프의 특성 방정식을 조작하여 제어 루프 내부 파라미터 변동에 따른 근궤적 작도법 설명 문서.

---

## 🎛️ 2. 상태 공간 기반 현대 제어 (`state-space/`)

물리적 시스템의 모든 내부 상태 변수(State Vector, $x$)를 행렬식 형태로 기술하고, 다변수 제어(MIMO) 및 최적 제어가 가능한 현대 제어 기법을 체계적으로 다룹니다.

### 💻 실습 스크립트 (MATLAB Scripts)
* **[control_demo_5_statespace.m](./state-space/control_demo_5_statespace.m):** 현대 제어 이론의 4단 빌드업(극점배치 ➡️ LQR 최적제어 ➡️ 상태관측기 결합 ➡️ 칼만필터 LQG 제어) 과정을 코드로 구현하여 노이즈와 추정 오차가 제어계에 미치는 영향을 완벽하게 대조 시뮬레이션합니다.
* **[lqr_5sec_design.m](./state-space/lqr_5sec_design.m):** 가중치 행렬 $Q, R$ 튜닝의 번거로움을 해결하기 위해, 5초 정착 시간 만족을 향해 LQR 게인을 자동으로 최적 반복 업데이트하는 스마트 루프 스크립트입니다.
* **[create_simulink_models.m](./state-space/create_simulink_models.m):** 시뮬링크 연동 실습에 필요한 정상 모델 및 적분기 모델(.slx)을 프로그래밍 방식으로 자동 생성하는 유틸리티입니다.
* **[state_feedback_with_integral_ex.m](./state-space/state_feedback_with_integral_ex.m):** 시뮬링크 연동 상태 피드백 및 적분 제어를 구동하고, 외란 환경 하에서의 정상상태 강인성 추종 특성을 비교 분석하는 실습 코드입니다.

### 📚 강의 및 설명 문서 (Lecture Documents)
* **[state_space_lecture.md](./state-space/state_space_lecture.md):** 4단계 현대 제어 기법(극점 배치, LQR, Observer, LQG)의 수학적 연계성과 개념의 진화를 블록선도와 시뮬레이션 데이터를 활용해 일목요연하게 풀어낸 빌드업 강의 노트입니다.
* **[pole_placement.md](./state-space/pole_placement.md):** 아커만(Ackermann) 공식을 활용한 극점 배치법과, 정상상태 추종 오차를 0으로 맞추기 위한 사전 스케일러 $\bar{N}$의 수학적 유도 과정을 상세히 다룹니다.
* **[lqr_tutorial.md](./state-space/lqr_tutorial.md):** 리카티 방정식(Riccati Equation)을 통한 최적화 원리와 Bryson Rule을 이용한 가중치 행렬 설정 실무 가이드를 제공합니다.
* **[state_feedback_integral_control.md](./state-space/state_feedback_integral_control.md):** 상태 피드백 적분 제어의 필요성, 수학적 수식 유도, 블록선도 및 MATLAB/Simulink 연동 시뮬레이션 방법론을 총망라한 종합 실습 가이드북입니다.
* **[modern_control_design_guide.md](./state-space/modern_control_design_guide.md):** 극점 배치 및 최적 LQR 제어계 설계를 돕는 MATLAB 현대 제어 실무 요약본 가이드입니다.

---

## 🛠️ 실습 활용 및 학습 방법 (How to Learn)

1. **이론 학습:** 각 디렉토리의 강의 문서(`lecture_notes.md`, `state_space_lecture.md`)를 먼저 읽어 개념적 뼈대를 잡습니다.
2. **실습 진행:** 매트랩에서 실습용 `.m` 스크립트를 열고 F5 키(또는 섹션 실행)를 눌러 실행합니다.
3. **가중치 및 게인 튜닝:** `Q`, `R`, PID `Kp/Ki/Kd` 등의 파라미터를 임의로 수정해보며 과도 응답과 보드선도의 대역폭이 어떻게 변하는지 눈으로 직관적으로 관찰합니다.
4. **가이드라인 참조:** 결과 분석에 막힐 때는 실습 가이드 문서(`walkthrough.md`, `lqr_tutorial.md`)를 함께 띄워두고 성능 사양과 매칭하며 깊이 있게 복습합니다.
