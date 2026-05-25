# 🌐 경희대학교 자동제어공학 강의자료실 (Class Automatic Control)

Welcome! 본 사이트는 경희대학교 전자공학과 김동한 교수님의 **자동제어공학(Automatic Control Engineering)** 과목 강의자료실입니다.
교재(Kuo의 자동제어 10판)와 관련된 각 주차별 이론 강의 자료(PDF) 및 매트랩(MATLAB)을 이용한 고전·현대 제어 실습 콘텐츠를 한눈에 볼 수 있도록 구성되어 있습니다.

---

## 👨‍🏫 강사 및 교과 정보 (Course Overview)

* **강사:** 전자공학과 김동한 교수 (전자정보대학 609호)
* **연락처:** donghani@khu.ac.kr | 내선번호 3831
* **상담시간:** 수업 후 1시간
* **주교재:** **Kuo의 자동제어 10판** (Farid Golnaraghi & Benjamin C. Kuo 저)
* **보조자료:** [유튜브 강의 채널 (경희대 김동한)](https://www.youtube.com/channel/UCT_h-5YhlC0t9LEdVckdrXQ)
* **강의 업로드:** [경희대학교 e-Campus](https://e-campus.khu.ac.kr) 및 본 GitHub 저장소

---

## 📚 1. 주차별 이론 강의 교안 (Lecture Slides)

아래 교안 링크를 클릭하여 각 챕터의 PDF 파일을 편리하게 내려받으실 수 있습니다.

* 📢 [**0. 자동제어 과목 소개**](./강의자료/0%20%EC%9E%90%EB%8F%99%EC%A0%9C%EC%96%B4%20%EA%B3%BC%EB%AA%A5%20%EC%86%8C%EA%B0%9C.pdf)
  * 과목 전반의 오리엔테이션 및 평가 가이드라인
* 📖 [**제1장. 제어 시스템 서론**](./강의자료/Kuo%EC%9D%98%EC%9E%90%EB%8F%99%EC%A0%9C%EC%96%B410e_01%EC%9E%A5.pdf)
  * 제어공학의 정의, 역사, 실생활 응용 분야 소개
* 📖 [**제2장. 제어 시스템의 수학적 기초**](./강의자료/Kuo%EC%9D%98%EC%9E%90%EB%8F%99%EC%A0%9C%EC%96%B410e_02%EC%9E%A5.pdf)
  * 라플라스 변환, 전달함수 표현법, 전기·기계 물리계 모델링 수학 기초
* 📖 [**제3장. 블록선도와 신호흐름선도**](./강의자료/Kuo%EC%9D%98%EC%9E%90%EB%8F%99%EC%A0%9C%EC%96%B410e_03v3.pdf)
  * Mason의 이득 공식을 활용한 다중 루프 시스템 축소 및 전달함수 구하기
* 📖 [**제4장. 물리적 시스템의 수학적 모델링**](./강의자료/Kuo%EC%9D%98%EC%9E%90%EB%8F%99%EC%A0%9C%EC%96%B410e_04v03.pdf)
  * 모터, 센서, 열 및 유체 시스템의 상태 방정식과 모델 결합
* 📖 [**제5장. 시간 영역 분석 (State & Transient)**](./강의자료/Kuo%EC%9D%98%EC%9E%90%EB%8F%99%EC%A0%9C%EC%96%B410e_05v01.pdf)
  * 1차/2차 시스템의 과도 상태 사양(상승 시간, 정착 시간, 오버슈트) 해석
* 📖 [**제6장. 피드백 제어 시스템의 분석**](./강의자료/Kuo%EC%9D%98%EC%9E%90%EB%8F%99%EC%A0%9C%EC%96%B410e_06v02.pdf)
  * 제어계의 외란 억제, 감도 분석 및 루스-허위츠(Routh-Hurwitz) 안정도 판별법
* 📖 [**제7장. 근궤적 기법 (Root Locus)**](./강의자료/Kuo%EC%9D%98%EC%9E%90%EB%8F%99%EC%A0%9C%EC%96%B410e_07v1.pdf)
  * 피드백 이득 $K$ 변화에 따른 폐루프 극점의 궤적 작도법 및 설계 응용
* 📑 [**부록. Ziegler-Nichols PID 동조법**](./강의자료/Kuo%EC%9D%98%EC%9E%90%EB%8F%99%EC%A0%9C%EC%96%B410e_07%EB%B6%80%EB%A1%9D_Ziegler-Nichols-method.pdf)
  * 응답 및 주파수 특성을 활용한 PID 이득 설정 실무 기법
* 📖 [**제8장. 주파수 영역 분석**](./강의자료/Kuo%EC%9D%98%EC%9E%90%EB%8F%99%EC%A0%9C%EC%96%B410e_08v01.pdf)
  * 보드선도(Bode Plot), 나이퀴스트 안정도 판별법, 주파수 이득/위상 여유
* 📖 [**제9장. 피드백 제어 시스템 설계**](./강의자료/Kuo%EC%9D%98%EC%9E%90%EB%8F%99%EC%A0%9C%EC%96%B410e_09v01.pdf) 및 [**설계 심화**](./강의자료/Kuo%EC%9D%98%EC%9E%90%EB%8F%99%EC%A0%9C%EC%96%B410e_09v02.pdf)
  * Lead(진상), Lag(지상), Lead-Lag 보상기 설계와 PID 제어기 종합 구현
* 📖 [**제10장. 상태 공간 분석법**](./강의자료/Kuo%EC%9D%98%EC%9E%90%EB%8F%99%EC%A0%9C%EC%96%B410e_10v01.pdf)
  * 상태 벡터 표현식, 상태 천이 행렬의 계산 및 시스템의 가관측성/가제어성
* 📖 [**제11장. 상태 피드백 제어 설계**](./강의자료/Kuo%EC%9D%98%EC%9E%90%EB%8F%99%EC%A0%9C%EC%96%B410e_11v02.pdf)
  * 극점 배치법(Pole Placement), 전차수/최소차수 상태 관측기(Observer) 설계

---

## 🛠️ 2. MATLAB 실습 프로그램 소개

자동제어공학 교과에 등장하는 수학적 설계를 시각적으로 증명하고 구현해보는 고품질 실습실입니다. 

### 📈 고전 제어 (전달함수 기반)
* **PID 제어 설계 ([control_demo_1_pid.m](./MATLAB/transfer-function/control_demo_1_pid.m))**
  * P, PI, PID 제어 특성을 그래프를 보며 직관적으로 관찰합니다.
* **시스템 해석 ([control_demo_2_analysis.m](./MATLAB/transfer-function/control_demo_2_analysis.m))**
  * 시간/주파수 해석(Root Locus, Bode, Nyquist)을 통해 안정성을 평가합니다.
* **진상/지상 보상기 설계 ([control_demo_3_compensator.m](./MATLAB/transfer-function/control_demo_3_compensator.m))**
  * 주파수 응답 사양을 만족시키는 위상 보상 기술을 실습합니다.
* **노이즈 미분 필터 ([control_demo_4_noise_filter.m](./MATLAB/transfer-function/control_demo_4_noise_filter.m))**
  * 미분기 노이즈를 억제하는 저역통과필터(LPF) 연동 설계 실무를 배웁니다.
* **파라미터 근궤적 ([root_locus_k.m](./MATLAB/transfer-function/root_locus_k.m))**
  * 특정 시스템 매개변수 변동 시 극점 궤적 변화를 작도합니다.

### 🎛️ 현대 제어 (상태 공간 기반)
* **상태 공간 종합 실습 ([control_demo_5_statespace.m](./MATLAB/state-space/control_demo_5_statespace.m))**
  * 극점 배치 ➔ LQR 최적화 ➔ 관측기 설계 ➔ LQG 제어에 이르는 4단계 현대 제어 완벽 정복.
* **LQR 가중치 자동 튜닝 ([lqr_5sec_design.m](./MATLAB/state-space/lqr_5sec_design.m))**
  * 5초의 목표 정착 시간을 자동 만족시키도록 최적화 루프를 활용해 가중치 행렬 $Q, R$을 스마트 튜닝합니다.
* **시뮬링크 모델 생성 ([create_simulink_models.m](./MATLAB/state-space/create_simulink_models.m))**
  * 시뮬링크 연동 실습용 블록 다이어그램 모델 자동 생성 스크립트.
* **상태 피드백 및 적분 제어 ([state_feedback_with_integral_ex.m](./MATLAB/state-space/state_feedback_with_integral_ex.m))**
  * 시뮬링크 연동 상태 피드백 및 적분 제어 강인성 검증 코드.
* **적분 제어 및 시뮬링크 실습 가이드 ([state_feedback_integral_control.md](./MATLAB/state-space/state_feedback_integral_control.md))**
  * 상태 피드백과 적분 제어의 원리를 수식, 블록선도, 매트랩/시뮬링크 예제로 설명하는 상세 학습 가이드.

---

## 🚀 빠른 시작 가이드

로컬 터미널을 열고 아래 명령어를 순서대로 실행하시면 전체 레포지토리 중 핵심 학습 자료와 실습실만 선별적으로 다운로드하여 즉시 작업을 시작하실 수 있습니다.

```bash
# Sparse-checkout 모드로 복제 진행
git clone --filter=blob:none --sparse https://github.com/donghani/class-automatic-control.git
cd class-automatic-control

# MATLAB 실습실 및 주요 마크다운, 페이지 파일 활성화
git sparse-checkout set MATLAB README.md index.html index.md
```
