# STM32 짐벌+메카넘 프로젝트 20일 일정표

## 전체 일정 개요

| 일자 | 작업 | 검증 |
|------|------|------|
| Day 1 | 빵판 + GND/전원 레일 만들기 | 멀티미터로 단락 검사 |
| Day 2 | Nucleo 단독 + 시리얼 통신 | 부팅 메시지 |
| Day 3 | MPU6050 추가 | I2C 통신, 기울기 값 |
| Day 4 | 서보 1개 (M1) 추가 | 손으로 흔들면 보정 |
| Day 5 | 서보 2개 → 짐벌 완성 ⭐ | 컵+물 흔들기 테스트 |
| Day 6 | XL4015 6V 조정 + 전원 분리 | 멀티미터로 6V 확인 |
| Day 7 | LiPo 메인 전원 전환 | 짐벌이 LiPo로도 동작 |
| Day 8-9 | BTS7960 1개 + 모터 1개 (FL) | 단독 회전 |
| Day 10-11 | BTS7960 4개 + 모터 4개 | 메카넘 8방향 ⭐ |
| Day 12-13 | 라인센서 5채널 | 디버그 출력으로 확인 |
| Day 14 | 라인 추종 PID | 라인 따라 주행 ⭐ |
| Day 15-16 | 엔코더 1~4개 | 카운트 증가 |
| Day 17-20 | 통합 + 시연 연습 + 발표 자료 | 최종 시연 |

⭐ = 가장 중요한 검증 단계

---

## 매일 해야 할 일 (한 페이지 요약)

### Day 1 - 빵판 + GND/전원 레일
- 큰 빵판 1개 + 작은 빵판 1개 준비
- 빵판 GND 줄 양 끝 멀티미터로 연속음 확인
- 두 빵판 GND 점퍼선으로 연결
- Nucleo GND → 빵판 GND 점퍼선
- (멀티미터 없으면 Day 2에서 동작 확인하면서 검증)

### Day 2 - Nucleo 단독
- STM32CubeIDE 설치 (1.19.0)
- 새 프로젝트 (Board Selector → NUCLEO-F446RE)
- RCC HSE, SYS Serial Wire, HCLK 180MHz 설정
- USART2 Asynchronous 활성화
- Ctrl+S 코드 생성
- Hello World 코드 추가 → 빌드 → 플래시
- PuTTY/Tera Term으로 시리얼 확인
- ✅ "Tick: 0, 1, 2..." 출력

### Day 3 - MPU6050
- MPU6050 결선 (VCC→3V3, GND, SCL→PB8, SDA→PB9, AD0→GND)
- .ioc에 I2C1 활성화 (Fast Mode 400kHz)
- mpu6050.h/c 파일 추가
- MPU6050_Init + ReadAll 호출
- ✅ "MPU6050 OK" + 가속도/자이로 값 출력
- ✅ 보드 기울이면 값 변화

### Day 4 - 서보 1개 단독
- 서보 M1 임시 결선 (빨강→Nucleo 5V, 신호→D7)
- .ioc에 TIM1_CH1 추가 (PSC=179, ARR=19999)
- servo.h/c 파일 추가
- -30°→0°→+30° 반복 코드
- ✅ 서보 부드럽게 움직임
- ⚠ 테스트 후 빨강 빼두기 (Day 6에서 6V로 다시)

### Day 5 - 짐벌 완성 ⭐
- 짐벌 기구 조립 (베이스 + 기둥 + 외측프레임 + 플레이트)
- 서보 2개 부착 (조립 전 0도로 중립화)
- MPU6050 플레이트 위 부착
- .ioc에 TIM1_CH2 추가 (PA9, Pulse=1500)
- TIM7 추가 (PSC=179, ARR=4999, 200Hz 인터럽트)
- attitude.h/c, pid.h/c 통합
- main에 짐벌 통합 코드
- PID 튜닝 (Kp=4.0부터 시작, Kd=0.5, Ki=0.1)
- ✅ 손으로 흔들면 플레이트 수평 유지
- ✅ 컵+물 안 쏟아짐
- 📹 시연 영상 촬영

### Day 6 - XL4015 6V 조정
- LiPo 연결 → 멀티미터로 XL4015 OUT 측정
- 파란 가변저항 돌려서 정확히 6.0V
- 서보 빨강 → 6V 라인으로 이동
- ✅ 서보 2개 동시 동작

### Day 7 - LiPo 메인 전원
- 옵션 A: USB로 Nucleo, LiPo로 서보만 (권장)
- 옵션 B: Nucleo VIN(7~12V)에 LiPo 직결
- ✅ USB 없이 LiPo만으로 부팅
- ✅ 짐벌 정상 동작

### Day 8-9 - BTS7960 단일 모터
- FL의 BTS7960 결선:
  - VM+ → 11.1V, VM- → GND
  - VCC → 5V, GND → GND
  - R_EN, L_EN → 5V
  - RPWM → PWM/CS/D10, LPWM → D4
  - M+, M- → 모터 두 선
- .ioc에 TIM4 활성화 (PSC=0, ARR=8999, 20kHz)
- bts7960.h/c 통합
- SetSpeed(0.3) → Stop → SetSpeed(-0.3) 반복
- ⚠ 모터는 차체에 부착하지 말고 단독 테스트
- ✅ 정/역 회전 확인

### Day 10-11 - 메카넘 4륜 ⭐
- 메카넘 휠 X자 패턴 조립 (가장 중요!)
- 나머지 BTS7960 3개 결선:
  - FR: RPWM→PWM/D9, LPWM→PWM/D3
  - RL: RPWM→PWM/D6, LPWM→D2
  - RR: RPWM→PWM/D5, LPWM→PWM/MOSI/D11
- TIM6 추가 (PSC=179, ARR=9999, 100Hz)
- mecanum.h/c 통합 + UART 키보드 처리
- ✅ w/s 전진/후진
- ✅ a/d 좌/우 평행이동 (메카넘 핵심!)
- ✅ q/e 좌/우 회전

### Day 12-13 - 라인센서 5채널
- TCRT5000 5채널 결선 (VCC→5V, GND, D1~D5→A0~A4)
- .ioc에 5개 핀 GPIO_Input + Pull-up
- linesensor.h/c 통합
- 라인 트랙 그리기 (검정 절연테이프)
- ✅ 라인 중앙: position 0.00
- ✅ 왼쪽: 음수, 오른쪽: 양수

### Day 14 - 라인 추종 PID
- linefollow.h/c 통합
- 모드 3 (라인 추종 + 짐벌) 활성화
- LINE_BASE_SPEED=0.2로 시작
- Kp 튜닝 → Kd 추가
- 직선 → 곡선 순서로 테스트
- ✅ 라인 따라 자율 주행

### Day 15-16 - 엔코더 (선택)
- FL 엔코더 A상 → A5
- .ioc에 EXTI0 활성화 (Rising Edge)
- encoder.h/c 통합
- ✅ 회전 시 카운트 증가
- (시간 있으면 추가 엔코더)

### Day 17-20 - 통합 + 시연 + 발표
- Day 17: 통합 시연 (라인 추종 + 짐벌 + 컵 물)
- Day 18: PPT 1차 완성
- Day 19: 발표 리허설
- Day 20: 최종 점검 + 일찍 자기

---

## 핵심 어필 포인트 (SK하이닉스 면접용)

1. **다중 인터럽트 우선순위**: TIM7(짐벌 200Hz) > TIM6(차량 100Hz) > UART > EXTI
2. **상보 필터**: 가속도+자이로 융합 (α=0.98)
3. **Anti-windup PID**: 적분 누적 방지
4. **Derivative on Measurement**: D-kick 없음
5. **메카넘 역운동학 + 정규화**: saturation 방지
6. **모듈 분리 설계**: 9개 모듈
7. **결정론적 제어 주기**: 인터럽트+main 분리

---

## 절대 금지사항

1. ❌ LiPo 11.1V → Nucleo 5V 핀 직결 (보드 손상)
2. ❌ XL4015 6V 조정 안 하고 서보 연결 (서보 손상)
3. ❌ BTS7960의 R_EN, L_EN 미연결 (모터 안 돔)
4. ❌ 공통 GND 누락 (시스템 불안정)
5. ❌ 한 번에 다 결선하기 (디버깅 불가)
