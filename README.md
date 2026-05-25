# 🎓 자동제어공학 (Automatic Control Engineering)
> **경희대학교 전자공학과 김동한 교수님 강의 자료실**  
> 본 저장소는 자동제어공학 수업의 강의 자료(PDF) 및 MATLAB을 이용한 실습 코드(고전 제어 & 현대 제어)를 체계적으로 정리해놓은 공간입니다.

---

## 📢 과목 및 강사 정보 (Course Information)

| 구분 | 정보 및 링크 |
| :--- | :--- |
| **👨‍🏫 담당 교수** | **김동한 교수** (전자공학과, 전자정보대학 609호, 내선 3831) |
| **✉️ 이메일** | [donghani@khu.ac.kr](mailto:donghani@khu.ac.kr) |
| **⏱️ 상담 시간** | 매주 수업 종료 후 1시간 |
| **📖 주교재** | **Kuo의 자동제어 10판** (Farid Golnaraghi, Benjamin C. Kuo 저) |
| **🔗 강의 영상** | [YouTube 채널 (경희대 김동한)](https://www.youtube.com/channel/UCT_h-5YhlC0t9LEdVckdrXQ) |
| **💻 실습 포털** | [경희대학교 e-Campus](https://e-campus.khu.ac.kr) |

---

## 📚 주차별 강의 자료 (Lecture Slides)

아래 강의 자료(PDF)는 본 저장소의 루트 디렉토리에 포함되어 있어 즉시 다운로드하거나 확인할 수 있습니다.

| 장 / 주제 | 파일명 (다운로드 링크) | 주요 학습 내용 |
| :--- | :--- | :--- |
| **과목 소개** | [0 자동제어 과목 소개.pdf](./0%20%EC%9E%90%EB%8F%99%EC%A0%9C%EC%96%B4%20%EA%B3%BC%EB%AA%A5%20%EC%86%8C%EA%B0%9C.pdf) | 과목 개요, 평가 방식, 범위 및 제어의 기본 개념 |
| **제1장** | [Kuo의자동제어10e_01장.pdf](./Kuo%EC%9D%98%EC%9E%90%EB%8F%99%EC%A0%9C%EC%96%B410e_01%EC%9E%A5.pdf) | 제어 시스템의 역사 및 실생활 제어기 예시 |
| **제2장** | [Kuo의자동제어10e_02장.pdf](./Kuo%EC%9D%98%EC%9E%90%EB%8F%99%EC%A0%9C%EC%96%B410e_02%EC%9E%A5.pdf) | 수학적 기초 (라플라스 변환, 미분방정식 모델링) |
| **제3장** | [Kuo의자동제어10e_03v3.pdf](./Kuo%EC%9D%98%EC%9E%90%EB%8F%99%EC%A0%9C%EC%96%B410e_03v3.pdf) | 블록선도(Block Diagram) 및 신호흐름선도(SFG) |
| **제4장** | [Kuo의자동제어10e_04v03.pdf](./Kuo%EC%9D%98%EC%9E%90%EB%8F%99%EC%A0%9C%EC%96%B410e_04v03.pdf) | 물리적 시스템의 수학적 모델링 (기계, 전기, 열) |
| **제5장** | [Kuo의자동제어10e_05v01.pdf](./Kuo%EC%9D%98%EC%9E%90%EB%8F%99%EC%A0%9C%EC%96%B410e_05v01.pdf) | 제어 시스템의 시간 영역 분석 (과도 및 정상상태 응답) |
| **제6장** | [Kuo의자동제어10e_06v02.pdf](./Kuo%EC%9D%98%EC%9E%90%EB%8F%99%EC%A0%9C%EC%96%B410e_06v02.pdf) | 선형 피드백 제어 시스템의 시간 영역 성능 사양 |
| **제7장** | [Kuo의자동제어10e_07v1.pdf](./Kuo%EC%9D%98%EC%9E%90%EB%8F%99%EC%A0%9C%EC%96%B410e_07v1.pdf) | 근궤적 기법 (Root Locus Method) 기본 원리 |
| **부록** | [Ziegler-Nichols Method.pdf](./Kuo%EC%9D%98%EC%9E%90%EB%8F%99%EC%A0%9C%EC%96%B410e_07%EB%B6%80%EB%A1%9D_Ziegler-Nichols-method.pdf) | PID 게인 동조를 위한 Ziegler-Nichols 휴리스틱 기법 |
| **제8장** | [Kuo의자동제어10e_08v01.pdf](./Kuo%EC%9D%98%EC%9E%90%EB%8F%99%EC%A0%9C%EC%96%B410e_08%EC%9E%A5.pdf) | 주파수 영역 분석 (Bode Plot, Nyquist Criterion) |
| **제9장** | [Kuo의자동제어10e_09v01.pdf](./Kuo%EC%9D%98%EC%9E%90%EB%8F%99%EC%A0%9C%EC%96%B410e_09v01.pdf) <br> [Kuo의자동제어10e_09v02.pdf](./Kuo%EC%9D%98%EC%9E%90%EB%8F%99%EC%A0%9C%EC%96%B410e_09v02.pdf) | 제어 시스템의 설계 (Lead, Lag, PID 보상기 설계) |
| **제10장** | [Kuo의자동제어10e_10v01.pdf](./Kuo%EC%9D%98%EC%9E%90%EB%8F%99%EC%A0%9C%EC%96%B410e_10v01.pdf) | 상태 공간 분석 (State Space Analysis) 및 좌표 변환 |
| **제11장** | [Kuo의자동제어10e_11v02.pdf](./Kuo%EC%9D%98%EC%9E%90%EB%8F%99%EC%A0%9C%EC%96%B410e_11v02.pdf) | 상태 피드백 제어 및 상태 관측기 설계 (Observer Design) |

---

## 🛠️ MATLAB 자동제어 실습실 (`/MATLAB`)

자동제어의 핵심인 **고전 제어(전달함수)**와 **현대 제어(상태 공간)**를 시각적으로 실습하고 깊게 학습할 수 있는 디렉토리입니다.

> 💡 **상세한 실습 가이드는 [MATLAB 종합 안내서](./MATLAB/README.md)를 참고해 주세요.**

### 📈 1. 전달함수 기반 고전 제어 ([/MATLAB/transfer-function](./MATLAB/transfer-function))
* **[control_demo_1_pid.m](./MATLAB/transfer-function/control_demo_1_pid.m):** PID 게인 변경에 따른 오버슈트, 정상상태 오차 대조.
* **[control_demo_2_analysis.m](./MATLAB/transfer-function/control_demo_2_analysis.m):** Poles/Zeros, Step, Root Locus, Bode, Nyquist 통합 분석.
* **[control_demo_3_compensator.m](./MATLAB/transfer-function/control_demo_3_compensator.m):** Lead/Lag 보상기(Compensator) 설계 실습.
* **[control_demo_4_noise_filter.m](./MATLAB/transfer-function/control_demo_4_noise_filter.m):** 실용적 미분기($\frac{s}{\tau s + 1}$) 및 LPF 결합 노이즈 저감.
* **[root_locus_k.m](./MATLAB/transfer-function/root_locus_k.m):** 시스템 설계 파라미터 변동에 따른 근궤적 작도.
* **학습 문서:** [고전제어 이론 강의노트](./MATLAB/transfer-function/lecture_notes.md), [실습 가이드북](./MATLAB/transfer-function/walkthrough.md), [미분 노이즈 심화 자료](./MATLAB/transfer-function/noise_and_derivative_filter.md)

### 🎛️ 2. 상태 공간 기반 현대 제어 ([/MATLAB/state-space](./MATLAB/state-space))
* **[control_demo_5_statespace.m](./MATLAB/state-space/control_demo_5_statespace.m):** 극점 배치 ➡️ LQR 최적제어 ➡️ 상태관측기 ➡️ LQG 제어 종합 비교.
* **[lqr_5sec_design.m](./MATLAB/state-space/lqr_5sec_design.m):** 5초 정착 시간을 자동 만족시키는 LQR 가중치 최적 자동 튜닝 루프.
* **학습 문서:** [현대제어 빌드업 강의록](./MATLAB/state-space/state_space_lecture.md), [LQR 튜닝 가이드](./MATLAB/state-space/lqr_tutorial.md), [사전 스케일러 $\bar{N}$ 유도 및 극점배치](./MATLAB/state-space/pole_placement.md)

---

## 🚀 시작하기 (Quick Start)

### 1. 로컬 저장소 내려받기 (Sparse Checkout 적용)
특정 대용량 폴더(`MATLAB` 실습 폴더)만 가볍게 다운로드하고 싶은 경우 아래 명령어를 순서대로 실행하세요.

```powershell
# 저장소 메타데이터만 다운로드
git clone --filter=blob:none --sparse https://github.com/donghani/class-automatic-control.git
cd class-automatic-control

# MATLAB 실습 폴더 및 기본 문서 활성화
git sparse-checkout set MATLAB README.md index.html
```

### 2. 매트랩 실행 및 실습
1. **MATLAB**을 실행하고 `class-automatic-control/MATLAB` 폴더로 작업 경로를 지정합니다.
2. 각 실습 폴더의 `.m` 스크립트를 열어 실행(`F5`)한 뒤, 마크다운(`*.md`) 강의 가이드 문서를 읽으며 파라미터를 튜닝해 봅니다.
