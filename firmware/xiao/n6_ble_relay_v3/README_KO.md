# N6 드론 XIAO BLE 중계 펌웨어 v3

묶은 날: 2026-11-25

이 묶음은 드론의 XIAO nRF52840 보드에 올리는 BLE 중계 펌웨어 v3입니다.
아래 순서는 프로젝트 문서 `QUICKSTART_KO.md` 6장과 8장에 적힌 것을 옮긴 것입니다.

> **보드에 펌웨어를 올리기 전에 팀에 먼저 알리세요.**

---

## 1. 무엇을 고쳤나

- **v1의 문제**: FC → 노트북 방향 데이터가 빠집니다. 약 1 kB/s부터 메시지 가운데가 빠져 CRC가 깨집니다. HEARTBEAT는 약 70%가 빠집니다.
- **v3에서 고친 것**: 받는 일과 보내는 일을 나누고 4096바이트 버퍼를 두었습니다.
- 그대로인 것: BLE 이름(`N6-DRONE-<번호>`), 핀, LED 규칙, 노트북 → FC 방향.
- 원인과 고친 내용은 소스 파일 맨 위 주석에 자세히 있습니다.

## 2. 지금 상태 (2026-11-25)

- v3는 **N6-DRONE-4에만** 올렸습니다(2026-11-25). **나머지 기체는 아직 v1입니다.**
- N6-DRONE-4에서 확인한 것: **2 kB/s까지 오류 0**.
  - ATTITUDE 10, 25, 50 Hz(428, 1024, 2023 B/s)에서 각 10초 동안 CRC 오류 0, 빠진 메시지 0.
- 아직 확인하지 않은 것:
  - `n6_fly.py` 전체 순서
  - 비행 중 동작
  - Windows 노트북과의 연결
  - Windows에서 XIAO에 올리는 것(아직 아무도 해 보지 않았습니다)
- 1, 2, 3, 5, 6번용 파일은 보드에 올려 본 적이 없습니다.
  - 이 묶음을 만들 때 비교해 보니, 4번용 파일과 기체 번호 1바이트만 다릅니다.

## 3. 들어 있는 파일

```
n6_ble_relay_v3/
├─ README_KO.md                      이 파일
├─ n6_ble_relay/
│   └─ n6_ble_relay.ino              v3 소스 (Arduino 스케치)
└─ dfu/
    ├─ n6_ble_relay_v3_id1_dfu.zip   N6-DRONE-1 용
    ├─ n6_ble_relay_v3_id2_dfu.zip   N6-DRONE-2 용
    ├─ n6_ble_relay_v3_id3_dfu.zip   N6-DRONE-3 용
    ├─ n6_ble_relay_v3_id4_dfu.zip   N6-DRONE-4 용
    ├─ n6_ble_relay_v3_id5_dfu.zip   N6-DRONE-5 용
    ├─ n6_ble_relay_v3_id6_dfu.zip   N6-DRONE-6 용
    └─ SHA256.txt                    위 6개 파일의 SHA-256 값
```

- **`dfu` 폴더 안의 zip은 풀지 마세요.** 올릴 때 zip 그대로 씁니다.
- 업로드용 스크립트는 따로 없습니다. 아래 명령을 직접 실행합니다.
- 파일이 손상됐는지 보려면 SHA-256 값을 `SHA256.txt`와 비교하세요. PowerShell에서는 `Get-FileHash <파일>`로 볼 수 있습니다(이 명령은 프로젝트 문서에는 없습니다).

## 4. 기체 번호에 맞는 파일 고르기

- 기체마다 BLE 이름이 다릅니다. 이름은 `N6-DRONE-<번호>`입니다.
- 파일 이름의 `id<번호>`가 기체 번호입니다.

  | 기체 BLE 이름 | 올릴 파일 |
  |---|---|
  | N6-DRONE-1 | `dfu\n6_ble_relay_v3_id1_dfu.zip` |
  | N6-DRONE-2 | `dfu\n6_ble_relay_v3_id2_dfu.zip` |
  | N6-DRONE-3 | `dfu\n6_ble_relay_v3_id3_dfu.zip` |
  | N6-DRONE-4 | `dfu\n6_ble_relay_v3_id4_dfu.zip` |
  | N6-DRONE-5 | `dfu\n6_ble_relay_v3_id5_dfu.zip` |
  | N6-DRONE-6 | `dfu\n6_ble_relay_v3_id6_dfu.zip` |

- 기체 번호를 모르면 올리기 전에 BLE 이름을 확인하세요. v3로 바꿔도 이름은 그대로입니다.
  - 프로젝트 폴더가 있으면 드론 배터리를 연결하고 스캔합니다(연결은 하지 않음, `QUICKSTART_KO.md` 8-1).
    ```powershell
    cd <압축 푼 곳>\stm32_projects\tools
    .venv\Scripts\python dev\ble_scan.py      # "N6-DRONE-4  ...  RSSI -50 dBm" 처럼 나옴
    ```
- **다른 번호의 파일을 올리면 그 기체의 BLE 이름이 바뀝니다.** 같은 이름의 기체가 두 대가 되지 않게 주의하세요.

## 5. 올리는 방법 A: 미리 빌드한 DFU 파일 (Arduino 없이)

출처: `QUICKSTART_KO.md` 6장.
**주의: 이 방법의 Windows용 설명은 8장에 없습니다.** 아래 명령은 6장에 적힌 그대로입니다.

1. XIAO를 **캐리어에서 빼고** USB-C로 PC에 연결합니다.
2. 도구를 설치합니다.
   ```
   pip install adafruit-nrfutil
   ```
3. XIAO 리셋 버튼을 **두 번** 눌러 부트로더로 넣습니다.
4. 올립니다. `id4`는 내 기체 번호로 바꾸세요.
   ```
   adafruit-nrfutil dfu serial -pkg n6_ble_relay_v3_id4_dfu.zip -p <포트> -b 115200 --singlebank
   ```
   - `-pkg` 뒤에는 `dfu` 폴더 안의 파일을 적습니다.
   - `<포트>`는 XIAO가 잡힌 시리얼 포트입니다. Windows에서 포트 이름을 찾는 법은 문서에 없습니다. (8-4는 FC USB가 `COM<숫자>`로 잡힌다고만 적었습니다.)
5. 끝나면 이 파일의 7번 "올린 뒤 확인"을 합니다.

## 6. 올리는 방법 B: Arduino IDE 2 (소스에서 빌드)

출처: `QUICKSTART_KO.md` 6장 "올리는 법" 1~5번, 8-5.
8-5에는 "Arduino IDE 2 Windows판에서 순서는 같습니다"라고 적혀 있습니다.

1. **File → Preferences → Additional boards manager URLs**에 다음 주소를 넣습니다.
   `https://files.seeedstudio.com/arduino/package_seeeduino_boards_index.json`
2. 보드 매니저에서 **"Seeed nRF52 Boards"**를 설치합니다. **mbed가 붙은 쪽이 아닙니다.** Bluefruit 라이브러리가 같이 들어 있습니다.
3. 보드로 **Seeed XIAO nRF52840**을 고릅니다.
4. `n6_ble_relay\n6_ble_relay.ino`를 열고, 파일 맨 위의 `DRONE_ID`를 **내 기체 번호로** 맞춥니다.
   - **소스에는 1로 적혀 있습니다.** 그대로 올리면 이름이 `N6-DRONE-1`이 됩니다.
5. XIAO를 **캐리어에서 빼고** USB-C로 PC에 연결한 뒤 업로드합니다.
   - 업로드가 안 되거나 포트가 안 보이면 리셋 버튼을 빠르게 두 번 누르세요(부트로더 모드). 부트로더 드라이브가 나타납니다.

## 7. 올린 뒤 확인

출처: `QUICKSTART_KO.md` 6장 "올리는 법" 6~7번, 4장, 8-1, 8-2. 시리얼 출력 형식은 소스 맨 위 주석.

1. XIAO를 USB에 꽂은 채로 시리얼 모니터(115200)를 엽니다.
   - 켜질 때 `N6 BLE relay v3 N6-DRONE-4`처럼 나옵니다. **v3인지, 번호가 맞는지** 보세요. 켜질 때 한 번만 나오므로 놓칠 수 있습니다.
   - 노트북 스크립트가 연결되면 `BLE connected`가 나옵니다.
   - 연결 중에는 5초마다 `stat rx=.. sent=.. notif=.. retry=.. lost=.. mtu=.. ci=..`가 나옵니다. **`lost`가 0이어야 합니다.**
2. XIAO를 캐리어에 다시 꽂습니다.
3. 드론 배터리를 연결하고 연결 시험을 합니다(`--no-arm`: 모터 안 돎).
   ```powershell
   cd <압축 푼 곳>\stm32_projects\tools
   .venv\Scripts\python n6_fly.py --no-arm --name N6-DRONE-4
   ```
   - `--name`을 빼면 `N6-DRONE-1`에 연결합니다(`README_KO.md` 6장).
   - 정상이면 몇 초 안에 `BLE 연결됨`, `FC HEARTBEAT 수신`, `노트북→FC 방향 확인됨 (COMMAND_ACK 수신)`, `[상태] BLE OK | 링크 0.1s | ...` 줄이 나옵니다.
   - `링크` 값이 1초 안쪽이면 정상입니다.
   - XIAO LED는 파랑과 초록이 켜집니다.
   - 끝낼 때는 `q`를 누릅니다.

알아 둘 것:
- **Windows 설정 → 블루투스에서 XIAO를 "장치 추가"로 페어링하지 마세요.** 스크립트가 직접 연결합니다.
- 펌웨어를 바꾼 뒤 연결이 이상하면, Windows 설정에서 장치를 제거해 남은 연결 정보를 지우세요.
- XIAO는 드론 배터리를 연결해야 BLE가 안정적입니다.
- XIAO는 BLE 연결을 하나만 받습니다. 다른 프로그램이나 휴대폰이 연결돼 있으면 찾지 못합니다.

## 8. v1으로 되돌리기

- 이 묶음에는 v1 파일이 없습니다.
- 되돌릴 v1 DFU 파일은 **4번용만** 있습니다. 프로젝트의 `tools\xiao\build\n6_ble_relay_v1_id4_dfu.zip`입니다. 올리는 법은 이 파일의 5번과 같습니다.
- 다른 번호용 v1 DFU 파일은 만들어 두지 않았습니다. v1은 원래 문서 "XIAO BLE 중계 펌웨어 올리기"의 펌웨어입니다.

## 9. 문서에 없어서 여기 쓰지 못한 것

- Windows에서 `adafruit-nrfutil`을 설치하고 실행하는 자세한 방법(가상환경 사용 여부, 포트 이름 찾기).
- XIAO용 Windows 드라이버가 따로 필요한지.
- DFU 파일을 올렸을 때 화면에 나오는 정상 출력 예.

Windows에서 처음 해 본 분은 결과를 `QUICKSTART_KO.md` 8장에 적어 주세요.
