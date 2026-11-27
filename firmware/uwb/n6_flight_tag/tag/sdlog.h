/* SD 카드 기록: 줄을 메모리에 쌓아 두고, 한 섹터씩 인터럽트 방식으로 쓴다. 쓰기를 기다리지 않는다.
 *
 * 카드: 캐리어 보드의 microSD (SDMMC2 4비트, 전원 켜기 PB9, 카드 감지 PB8).
 * 파일: 카드 맨 위 폴더의 UWBLOG.BIN (PC 에서 tools/sd_prepare.py 로 미리 만든다. 0 으로 채운 큰 파일).
 *       펌웨어는 그 파일이 차지한 섹터에 차례로 쓴다. 파일 시스템은 고치지 않는다.
 * 내용: 글자 그대로의 줄들. 앞 기록 뒤에 이어 쓴다(아직 0 인 첫 섹터부터). 0.5 s 동안 한 섹터가 안 차면
 *       남은 자리를 줄바꿈으로 채워서 쓴다. 그래서 전원이 갑자기 꺼져도 잃는 것은 마지막 0.5 s 쯤이다.
 *
 * 2026-11-24 보드에서 잰 값 (-O0): 한 섹터 쓰기를 거는 데 0.28 ms, 그 뒤 끝날 때까지 주 루프가 멈추는 시간은 4 µs 이하.
 * 카드가 가끔 100 ms 넘게 붙잡고 있어도 그동안 기다리지 않는다(메모리에 쌓일 뿐).
 */
#ifndef SDLOG_H
#define SDLOG_H

#include <stdint.h>

enum sdlog_state {
    SDLOG_OFF = 0,     /* 카드나 파일이 없어 쓰지 않음 */
    SDLOG_IDLE,        /* 쓸 준비가 됨 */
    SDLOG_WRITING,     /* 쓰기를 걸어 둠 */
    SDLOG_WAIT_READY,  /* 자료는 다 보냈고 카드가 다음 명령을 받을 때를 기다림 */
    SDLOG_RECOVER,     /* 오류가 나서 카드를 껐다 켜야 함 */
    SDLOG_FULL,        /* 파일이 꽉 참 */
    SDLOG_FAILED,      /* 여러 번 실패해서 그만둠 */
};

struct sdlog_stats {
    uint8_t state;
    uint8_t card_ok;          /* 카드 초기화 성공 */
    uint8_t file_result;      /* enum sdfat_result */
    uint32_t init_error;      /* 카드 초기화 오류 코드 (HAL) */
    uint32_t first_block, file_blocks;
    uint32_t next_block;      /* 파일 안에서 다음에 쓸 섹터 */
    uint32_t written;         /* 이번에 켠 뒤 쓴 섹터 수 */
    uint32_t dropped_bytes;   /* 쌓을 자리가 없어 버린 글자 수 */
    uint32_t errors;          /* 쓰기 오류 횟수 */
    uint32_t max_write_us;    /* 쓰기를 건 때부터 카드가 준비될 때까지 가장 오래 걸린 시간 */
    uint32_t max_start_us;    /* 쓰기를 거는 함수 안에서 보낸 가장 긴 시간 */
    uint32_t queued;          /* 지금 쌓여 있는 섹터 수 */
};

/* 부팅 때 한 번. 카드를 켜고 파일을 찾는다(수백 ms 걸림). 기록할 수 있으면 1 */
int sdlog_init(void);

/* 시험용: 파일 대신 카드의 섹터 범위를 직접 준다. 카드는 sdlog_init 으로 이미 켜져 있어야 한다 */
int sdlog_use_region(uint32_t first_block, uint32_t blocks);

/* 한 줄을 쌓는다(끝의 줄바꿈까지 포함해서 준다). 기다리지 않는다 */
void sdlog_write(const char *text, unsigned length);

/* 패킷을 기다리는 동안 계속 부른다. may_start 가 참일 때만 새 쓰기를 걸거나 카드 상태를 묻는다
 * (그때 0.3 ms 쯤 걸릴 수 있으므로, 무선이 조용한 구간에서만 참으로 준다) */
void sdlog_service(int may_start);

/* 쌓인 것을 지금 다 쓴다(기다린다). 시험이나 끝낼 때만 */
void sdlog_flush_blocking(uint32_t timeout_ms);

const struct sdlog_stats *sdlog_stats(void);

/* 한 섹터 읽기 (sdfat 과 시험이 쓴다). 성공하면 0 */
int sdlog_read_block(uint32_t block, uint8_t *buffer);

#endif
