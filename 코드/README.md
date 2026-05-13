# STM32 Nucleo-F446RE 통합 프로젝트
## 짐벌 + 메카넘 4륜 + 라인 트레이서

### 사용 도구: **STM32CubeIDE 단독** (다른 프로그램 불필요)

> CubeMX는 CubeIDE 안에 내장돼 있습니다. 별도 설치 X.
> 프로젝트 안의 .ioc 파일을 더블클릭하면 옛날 CubeMX 화면이 그대로 열립니다.

---

## 1. 시스템 개요

차량(메카넘 4륜) 위에 짐벌(2축)을 올리고, 그 위에 컵을 올림.
바닥의 검은 라인을 따라 차가 주행해도 짐벌이 수평을 유지해서 컵 안의 물이 안 쏟아짐.

---

## 2. 부품 목록

| 부품 | 수량 | 비고 |
|------|------|------|
| STM32 Nucleo-F446RE | 1 | 메인 MCU |
| MPU6050 (GY-521) | 1 | 짐벌용 IMU, I2C |
| MG996R 서보 | 2 | 짐벌 Pitch + Roll |
| JGB37-520 + 엔코더 | 4 | 메카넘 휠 구동 |
| BTS7960 H-브리지 | 4 | 모터당 1개씩 |
| 메카넘 휠 | 4 | 위에서 볼 때 X 패턴으로 조립 |
| XL4015 벅 컨버터 | 1 | 11.1V → 6V (서보 전원) |
| LiPo 11.1V 2200mAh 3S | 1 | 메인 배터리 |
| TCRT5000 5채널 라인 모듈 | 1 | 디지털 출력 |

---

## 3. 파일 구조

```
Project/
 ├ Inc/
 │   ├ mpu6050.h           ← MPU6050 I2C 통신
 │   ├ attitude.h          ← 상보필터 자세 추정
 │   ├ pid.h               ← 범용 PID 제어기
 │   ├ servo.h             ← MG996R PWM 제어
 │   ├ bts7960.h           ← BTS7960 단일 모터 제어
 │   ├ mecanum.h           ← 메카넘 4륜 운동학
 │   ├ linesensor.h        ← 5채널 라인 위치 계산
 │   ├ linefollow.h        ← 라인 추종 PID
 │   └ encoder.h           ← 엔코더 EXTI 카운트
 ├ Src/
 │   ├ (위 헤더 대응 .c 9개)
 │   └ main_user_code.c    ← ⚠ 별도 파일 아님! main.c 복붙 가이드
 └ README.md
```

### ⚠ main_user_code.c 는 별도 파일이 아닙니다!
CubeIDE가 생성한 `main.c` 의 `/* USER CODE BEGIN xxx */` 블록에 섹션별로 복붙하는 가이드입니다.

---

## 4. CubeIDE 단계별 가이드

### 4.1 STM32CubeIDE 설치
1. https://www.st.com/en/development-tools/stm32cubeide.html
2. 회원가입 (무료) → 다운로드 → 설치
3. ST-LINK 드라이버는 자동 설치됨

### 4.2 새 프로젝트 만들기
1. CubeIDE 실행
2. **File → New → STM32 Project**
3. **Board Selector** 탭 → 검색창에 `NUCLEO-F446RE` 입력 → 선택 → **Next**
4. 프로젝트 이름: `Gimbal_Mecanum` (또는 원하는 이름) → **Finish**
5. "Initialize all peripherals with their default Mode?" 팝업 → **No**
6. .ioc 파일이 자동으로 열림 (이게 옛날 CubeMX 화면)

### 4.3 .ioc 화면에서 설정 (왼쪽 트리메뉴 사용)

#### ① RCC, SYS (가장 먼저)
- **System Core → RCC**
  - HSE: **Crystal/Ceramic Resonator**
- **System Core → SYS**
  - Debug: **Serial Wire** (이게 안 되면 ST-LINK 디버그 불가)

#### ② Clock Configuration 탭 (.ioc 상단 탭)
- 오른쪽 끝 **HCLK** 입력란에 **180** 입력 → Enter → "OK" / "Yes"
- 자동으로 PLL 계산해서 180MHz로 맞춰줌

#### ③ Pinout & Configuration 탭 → 핀 설정

**칩 그림에서 직접 핀 클릭하여 기능 선택**

##### 짐벌 그룹
| 핀 클릭 → 선택 | 기능 |
|----------------|------|
| PB8 | I2C1_SCL |
| PB9 | I2C1_SDA |
| PA8 | TIM1_CH1 |
| PA9 | TIM1_CH2 |

##### 메카넘 4륜 PWM 그룹
| 핀 | 기능 | 신호 |
|----|------|------|
| PC6 | TIM8_CH1 | FL_RPWM |
| PC7 | TIM8_CH2 | FR_RPWM |
| PC8 | TIM8_CH3 | RL_RPWM |
| PC9 | TIM8_CH4 | RR_RPWM |
| PB6 | TIM4_CH1 | FL_LPWM |
| PB7 | TIM4_CH2 | FR_LPWM |
| PA0 | TIM5_CH1 | RL_LPWM (※ 엔코더와 겹치면 다른 핀) |
| PA1 | TIM5_CH2 | RR_LPWM |

> ⚠ 핀이 노란색/빨간색이면 다른 기능과 충돌. 그땐 CubeIDE가 충돌 정보를 보여주므로 다른 핀으로 옮기면 됨. **충돌 없을 때까지 핀을 조정한 후 코드를 그 핀에 맞춰 수정**.

##### 엔코더 (EXTI 인터럽트)
| 핀 | 기능 |
|----|------|
| PA4 | GPIO_EXTI4 |
| PB10 | GPIO_EXTI10 |
| PB11 | GPIO_EXTI11 |
| PA15 | GPIO_EXTI15 |

각 핀에서 **우클릭 → GPIO_EXTI 선택**
이후 **System Core → GPIO** 들어가서 각 EXTI 핀:
- GPIO mode: **External Interrupt Mode with Rising edge trigger**
- GPIO Pull-up/Pull-down: **Pull-up**

##### 라인 센서 (디지털 입력 5개)
| 핀 | 기능 |
|----|------|
| PC0 | GPIO_Input |
| PC1 | GPIO_Input |
| PC2 | GPIO_Input |
| PC3 | GPIO_Input |
| PC4 | GPIO_Input |

각 핀 GPIO 설정에서 Pull-up

##### 통신/디버그
| 핀 | 기능 | 비고 |
|----|------|------|
| PA2 | USART2_TX | ST-LINK 가상 COM |
| PA3 | USART2_RX | |
| PA5 | GPIO_Output | 보드 LED2 (이미 설정돼있음) |

#### ④ 타이머 상세 설정 (Pinout 탭의 왼쪽 트리에서)

##### TIM1 (짐벌 서보 PWM 50Hz)
- Clock Source: **Internal Clock**
- Channel 1: **PWM Generation CH1**
- Channel 2: **PWM Generation CH2**
- Parameter Settings:
  - Prescaler: **179**
  - Counter Period (ARR): **19999**
  - auto-reload preload: **Enable**
- PWM Generation CH1 / CH2:
  - Pulse: **1500** (서보 중립)

##### TIM8 (메카넘 RPWM 20kHz)
- Clock Source: Internal Clock
- Channel 1, 2, 3, 4: **PWM Generation**
- Prescaler: **0**, Counter Period: **8999**
- 모든 채널 Pulse: **0**

##### TIM4 (메카넘 LPWM 2채널)
- Clock Source: Internal Clock
- Channel 1, 2: PWM Generation
- Prescaler: **0**, Counter Period: **8999**
- Pulse: **0**

##### TIM5 (메카넘 LPWM 나머지 2채널)
- 동일하게 PSC=0, ARR=8999, Pulse=0

##### TIM6 (차량 100Hz 인터럽트)
- Activated ✔ (Clock Source는 자동으로 Internal)
- Prescaler: **179**, Counter Period: **9999** (10ms)
- **NVIC Settings 탭 → TIM6 global interrupt ✔ 체크**

##### TIM7 (짐벌 200Hz 인터럽트)
- Activated ✔
- Prescaler: **179**, Counter Period: **4999** (5ms)
- **NVIC Settings → TIM7 global interrupt ✔**

#### ⑤ I2C1
- Mode: **I2C**
- Parameter Settings:
  - I2C Speed Mode: **Fast Mode**, 400000 Hz
  - Primary Address Length: 7-bit

#### ⑥ USART2
- Mode: **Asynchronous**, 115200 8N1
- **NVIC Settings → USART2 global interrupt ✔** (수신 인터럽트용)

#### ⑦ NVIC 우선순위 (System Core → NVIC)
- Priority Group: **4 bits for pre-emption priority** 선택
- 우선순위:
  - TIM7 global interrupt: **1** (짐벌 최우선)
  - TIM6 global interrupt: **2**
  - EXTI line interrupts: **2**
  - USART2 global interrupt: **3**

#### ⑧ Project Manager 탭
- Code Generator → **Generate peripheral initialization as a pair of '.c/.h' files per peripheral** ✔ (체크)

### 4.4 코드 생성
- **.ioc 파일에서 Ctrl+S** (또는 Project → Generate Code)
- "Open Associated Perspective?" → Yes
- main.c, stm32f4xx_it.c 등이 자동 생성됨

### 4.5 소스 파일 추가
1. CubeIDE 왼쪽 Project Explorer:
   - `Core/Inc/` 폴더 우클릭 → Import → File System → 제공된 9개 헤더 복사
   - `Core/Src/` 폴더에 9개 .c 파일 복사
2. **main_user_code.c는 추가하지 말고** 내용만 main.c에 복붙

### 4.6 main.c 에 코드 복붙

CubeIDE의 main.c 열기 → 다음 영역들 찾기:

```c
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */

/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* USER CODE BEGIN PV */
/* USER CODE END PV */

/* USER CODE BEGIN PFP */
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
/* USER CODE END 0 */

int main(void) {
  ...
  /* USER CODE BEGIN 2 */
  /* USER CODE END 2 */

  while (1) {
    /* USER CODE END WHILE */
    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/* USER CODE BEGIN 4 */
/* USER CODE END 4 */
```

각 영역 사이에 `main_user_code.c` 에서 해당 섹션을 그대로 복사.

### 4.7 빌드 & 플래시
1. **Ctrl+B** (또는 망치 아이콘) → 빌드
2. 에러 0개 확인 (`Build Finished. 0 errors`)
3. **벌레 아이콘 (Debug)** 누르면 자동 플래시 + 디버그 시작
4. Resume(F8) 누르면 실행

### 4.8 시리얼 모니터
1. CubeIDE → Window → Show View → Terminal
2. Open Terminal → Serial Terminal
3. 포트: ST-LINK Virtual COM, Baud: **115200**
4. 부팅 메시지 확인

---

## 5. 회로 결선

### 5.1 전원 라인
```
LiPo 11.1V (+) ─┬─ XL4015 VIN+
                └─ BTS7960 × 4의 VM+ (모터 전원)

LiPo (-) ──── 공통 GND

XL4015 VOUT (6.0V로 정확히 조정) ─── MG996R 빨강 × 2
                                       (서보 전원만 6V, BTS7960 로직은 5V)
```

### 5.2 BTS7960 결선 (4개 동일)
| BTS7960 핀 | 연결 |
|------------|------|
| VM+ | LiPo + |
| VM- | 공통 GND |
| VCC | Nucleo 5V (로직용) |
| GND | 공통 GND |
| RPWM | TIM8 채널 핀 (예: PC6) |
| LPWM | TIM4/TIM5 채널 핀 (예: PB6) |
| R_EN | **Nucleo 5V 직결** (항상 ON) |
| L_EN | **Nucleo 5V 직결** (항상 ON) |
| R_IS, L_IS | 미사용 |
| M+, M- | JGB37-520 모터 두 선 |

### 5.3 JGB37-520 엔코더 (모터 케이블 6선)
```
빨강 (Motor+)  → BTS7960 M+
검정 (Motor-)  → BTS7960 M-
주황 (VCC)     → Nucleo 5V  (또는 3.3V, 데이터시트 확인)
초록 (GND)     → 공통 GND
노랑 (A상)     → STM32 EXTI 핀
파랑 (B상)     → 미사용 (또는 디버깅용)
```
> 색깔은 제조사마다 다를 수 있습니다. 데이터시트 또는 모터에 붙은 라벨 확인.

### 5.4 MPU6050
| MPU 핀 | 연결 |
|--------|------|
| VCC | Nucleo 3.3V |
| GND | 공통 GND |
| SCL | PB8 |
| SDA | PB9 |
| AD0 | GND |
| INT | 미사용 |

### 5.5 TCRT5000 5채널 라인 센서
| 핀 | 연결 |
|----|------|
| VCC | Nucleo 5V |
| GND | 공통 GND |
| D1~D5 (또는 OUT1~OUT5) | PC0~PC4 |

### 5.6 공통 GND
**LiPo - / XL4015 - / 모든 BTS7960 GND / Nucleo GND / 서보 GND / MPU GND / 센서 GND** 가 모두 **하나의 굵은 GND 레일**에 연결되어야 합니다. 안 그러면 노이즈로 동작 불안정 또는 PWM 오작동.

---

## 6. UART 명령 (Tera Term / CubeIDE Terminal)

| 키 | 동작 |
|----|------|
| `1` | 짐벌만 동작 (책상 위 테스트) |
| `2` | 짐벌 + 수동 차량 |
| `3` | **짐벌 + 라인 추종 (발표 시연)** |
| `0` | 전체 정지 |
| `w`/`s` | 전진/후진 (모드 2) |
| `a`/`d` | **왼쪽/오른쪽 평행이동** (메카넘 횡이동) |
| `q`/`e` | 제자리 좌/우회전 |
| 스페이스 | 모터 정지 |
| `r` | 엔코더 카운트 리셋 |

---

## 7. 한 달 작업 일정

### Week 1 — 짐벌 단독
- 9개 모듈 중 mpu6050/attitude/pid/servo + main 의 짐벌 부분만 살림
- TIM1/I2C1/TIM7/USART2 만 설정해서 빌드
- 책상 위에서 손으로 흔들었을 때 컵 수평 유지 확인
- PID 튜닝: Kp 키워 진동 직전 → Kd 추가 → Ki 마지막

### Week 2 — 메카넘 4륜
- BTS7960 결선
- TIM8 + TIM4 + TIM5 설정
- 모터 1개씩 단독 테스트 (`SetSpeed(0.3)` 으로 방향 확인)
- **WHEEL_xx_REVERSE 플래그 조정**
- 4륜 통합 → UART 키보드 동작

### Week 3 — 라인 트레이서 추가
- TCRT5000 결선
- 부팅 후 raw 값으로 LINE_ACTIVE_LOW 확정 (검정 위 = 0인지 1인지)
- LineFollow PID 튜닝 (base_speed 0.2부터)
- 곡선 라인 테스트
- 엔코더 카운트 정상 동작 확인

### Week 4 — 통합 + 발표
- 짐벌을 차 위에 마운트
- 컵 + 물 올리고 라인 주행 시연
- 발표 자료: 블록도, PID 응답 그래프, 상보필터 효과 비교
- 시간 남으면 칼만 필터 업그레이드, 엔코더 PID 추가

---

## 8. 흔한 실수 디버깅

| 증상 | 원인/해결 |
|------|----------|
| 부팅 메시지 안 나옴 | USART2 핀, baudrate 115200 확인 |
| MPU6050 init fail | AD0 핀, I2C 결선, 풀업 |
| 모터 아예 안 돔 | BTS7960 EN 핀 5V 미연결 |
| 한 모터만 안 돔 | 해당 BTS7960 PWM 핀 / TIM 설정 |
| 평행이동 안 됨 | **메카넘 휠 X 패턴 조립 확인** |
| 차가 회전하며 전진 | 한 바퀴 방향 반전 — `WHEEL_xx_REVERSE = 1` |
| 짐벌이 발진 | PID Kd 너무 큼, 또는 NVIC TIM7 우선순위 미적용 |
| 짐벌 잠깐씩 멈춤 | I2C 통신 충돌 — TIM7 인터럽트 우선순위 최우선으로 |
| 라인 추종 발진 | LINE_KP↓ 또는 base_speed↓ |
| 라인 추종 부호 반대 | LineFollow_Update의 wz 부호 또는 mecanum.h의 ω 방향 확인 |
| 시스템 재시작 반복 | 전류 부족, 공통 GND 부실 |

---

## 9. 발표 어필 포인트

- **다중 인터럽트 우선순위**: 짐벌(200Hz) > 차량(100Hz) > UART > EXTI
- **상보 필터** 의의: 가속도(저주파) + 자이로(고주파) 융합
- **메카넘 역운동학** + 정규화 (saturation 방지)
- **Anti-windup**: 외란 후 빠른 복귀
- **Derivative on Measurement**: 모드 전환 시 D-kick 없음
- **모듈 분리 설계**: 9개 모듈로 분리 → 재사용성 / 디버깅 용이

화이팅! 한 달이면 완성도 높게 만들 수 있어요.
SK하이닉스 꼭 합격하시길!
