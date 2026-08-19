# 🤖 Bin Picking Advancement
> 3D 비전-로봇 자동 캘리브레이션 및 포즈 추정 알고리즘 연구

<br>

## 📌 연구 개요
<img src="https://github.com/user-attachments/assets/def23c7d-29eb-4442-893e-70bf7f8344d3" width="350" align="right" alt="Bin Picking Robot"/>

본 연구는 생산 라인에서 활용되는 3D 비전 기반 빈 피킹(Bin Picking) 로봇의 **비전-로봇 좌표계 캘리브레이션 자동화** 및 **곡면 가공(Sanding)을 위한 툴 포즈(Tool Pose) 최적화**를 목적으로 진행되었습니다. (2024 서울대학교 공과대학 산학연계 프로젝트)

기존 수동 티칭(Teaching) 방식의 비효율성을 개선하고, 곡면 피킹 시 발생하는 직교성 오차 및 기구학적 한계를 수학적 모델링(SVD, Rodrigues' Formula)을 통해 분석 및 해결하고자 했습니다.

> ⚠️ **보안사항**: 본 프로젝트는 기업체와의 산학연계(NDA)로 진행되었으며, 민감한 기업 데이터 및 상세 스펙은 마스킹 처리하여 수록하였습니다.

<br> <!-- 이미지 정렬로 인한 레이아웃 겹침 방지 -->

---

## 📂 디렉토리 구조
프로젝트 핵심 파이프라인에 따라 구성된 알고리즘 모듈입니다.

*   `01-TransMat.ipynb`: 임의 이동 테스트 기반 카메라-로봇 4D 변환행렬(Calibration) 도출
*   `02-1-Rodrigues_General.ipynb`: Rodrigues' Formula 기반 3D 회전행렬 및 오일러 각 추출
*   `02-2-SVD_PlaneFitting.ipynb`: 3D Point Cloud 데이터(`Z.ply`) SVD 평면 피팅 및 법선벡터 추정
*   `03-HH020_TransMat.ipynb`: 6자유도 산업용 로봇(HH020) 순운동학(Forward Kinematics) 모듈 검증
*   `Z.ply`: 검증용 3D Point Cloud 원본 데이터

---

## 🛠️ 핵심 알고리즘 및 수학적 모델링

### 1. Teaching-less 비전-로봇 좌표계 자동 캘리브레이션
작업자의 숙련도에 의존하던 수동 티칭 방식의 한계를 극복하기 위해, **경험적 평행이동 기반의 변환행렬 산출 알고리즘**을 제안했습니다.
*   **알고리즘 원리**: 로봇 팔 좌표계에서 직교 3축($x, y, z$) 방향으로 지정된 거리($k$)만큼 평행이동하는 테스트를 수행합니다.
*   **4D 변환행렬 도출**: 이동 전후의 카메라 좌표계 측정값을 연립하여 로봇 좌표계에서 카메라 좌표계로 변환하는 $4 \times 4$ 동차변환행렬(Homogeneous Transformation Matrix) $A$ 및 역행렬 $A^{-1}$를 산출합니다.
*   **연구 결과**: [Issue #1](https://github.com/ben020410/2024W-ETS-BPA/issues/1). 카메라 측정 좌표만으로 로봇 팔의 제어 좌표를 실시간으로 정밀 역산하는 파이프라인을 확립했습니다.

### 2. 3D Point Cloud 기반 곡률 매칭 및 툴 회전각 제어
단순 위치(Position) 정렬을 넘어, 곡면 샌딩 공정의 핵심인 제품 곡면의 법선벡터와 로봇 툴(Tool)의 지향 벡터를 일치시키는 **자세(Orientation) 제어 파이프라인**을 구축했습니다.
*   **SVD 평면 피팅**: 카메라에서 취득한 Point Cloud 데이터에 특이값 분해(SVD)를 적용하여 최적의 표면 법선벡터를 추정합니다.
*   **Rodrigues' Rotation Formula**: 두 벡터 간의 회전축 $n$과 회전각 $\theta$ (계산 간소화를 위해 $\theta = \pi$로 설정)를 기반으로 회전행렬 $K$를 산출합니다.

$$v' = \left[ I + \sin\theta K + (1 - \cos\theta) K^2 \right] v, \quad K = \begin{bmatrix} 0 & -n_z & n_y \\ n_z & 0 & -n_x \\ -n_y & n_x & 0 \end{bmatrix}$$

*   **오일러 각 추출**: 산출된 회전행렬 $R$을 역삼각함수를 통해 로봇 제어기가 인식할 수 있는 Roll-Pitch-Yaw 각도($\alpha, \beta, \gamma$)로 변환합니다.
*   **연구 결과**: [Issue #2](https://github.com/ben020410/2024W-ETS-BPA/issues/2), [Issue #3](https://github.com/ben020410/2024W-ETS-BPA/issues/3). 법선벡터 산출 경향성은 이론값과 일치하였으나, 특정 각도에서 오차가 발생함을 확인하여 기구학적 구동 한계 분석으로 연구를 확장했습니다.

### 3. 6자유도(6-DOF) 로봇 기구학 분석 (HH020 모델)
2단계에서 발생한 제어 오차의 원인을 분석하기 위해, 대상 로봇(현대로보틱스 HH020)의 기구학적 특성을 모델링했습니다.
*   **순운동학(Forward Kinematics)**: 카탈로그 제원 및 HRSpace 시뮬레이션 데이터를 바탕으로, 6개 관절 각도를 입력받아 End-effector의 최종 위치 및 회전량을 연산하는 알고리즘을 자체 구현했습니다.
*   **오차 원인 규명**: 기존 제어 알고리즘은 목적지까지의 최단 회전량만 산출하므로, 각 관절의 **물리적 구동 제한 범위(Joint Limits)를 고려하지 못해 기구학적 특이점이나 충돌 오류가 발생**함을 수학적으로 입증했습니다.

---

## 🚀 향후 연구 과제
*   **역운동학(Inverse Kinematics) 구현**: 도출된 6-DOF 동차변환행렬을 기반으로, Joint Limit을 회피하여 목표 포즈에 도달할 수 있는 역운동학 경로 생성 알고리즘 고안.
*   **짐벌 락(Gimbal Lock) 문제 해결**: 3차원 오일러 각 회전 시 발생하는 특이점(Singularity) 문제를 방지하기 위해, **사원수(Quaternion)** 기반의 안정적인 자세 제어 로직 도입.

---

## 📚 참고 문헌
1. F. C. Park and K. M. Lynch. (2016). *Introduction to Robotics: Mechanics, Planning, and Control*. Northwestern University.
2. Michaela Borzechowski. (2017). *Best-Fit Subspaces and Singular Value Decomposition*. Wolfgang Mulzer.
3. Jamshed Iqbal, et al. (2012). *Modeling and Analyzing of a 6 DOF Robotic Arm Manipulator*. Canadian Journal on Electrical and Electronics Vol. 3, No. 6.
