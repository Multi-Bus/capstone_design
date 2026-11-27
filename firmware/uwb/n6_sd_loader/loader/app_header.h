/* N6APP.BIN 헤더 검사 (하드웨어와 무관: 보드의 로더와 PC 시험이 같이 쓴다). 형식은 DESIGN_KO.md 3장. */
#ifndef APP_HEADER_H
#define APP_HEADER_H

#include <stdint.h>

#define APP_HEADER_SIZE 32U
#define APP_HEADER_VERSION 1U
#define APP_LOAD_ADDR 0x34000400UL    /* AXISRAM1. 펌웨어를 --app-link 로 빌드하면 여기서 돈다 */
#define APP_MAX_LEN (512UL * 1024UL)
#define APP_RAM_LO 0x341C0000UL       /* 펌웨어의 변수와 스택 (지금과 같은 자리) */
#define APP_RAM_HI 0x34200000UL

struct app_header {
    uint16_t version;
    uint8_t tag_id;
    uint32_t load;
    uint32_t length;
    uint32_t crc;
    char name[13];                    /* 12글자 + 끝 0 */
};

enum app_check {
    APP_OK = 0,
    APP_BAD_MAGIC,
    APP_BAD_VERSION,
    APP_WRONG_ID,
    APP_BAD_LENGTH,
    APP_BAD_CRC,
    APP_BAD_VECTORS,
};

/* raw: 파일 앞 32바이트. file_bytes: 파일이 차지한 바이트 수(섹터 단위로 올림해도 됨) */
enum app_check app_check_header(const uint8_t raw[APP_HEADER_SIZE], uint8_t my_id, uint32_t file_bytes,
                                struct app_header *out);

/* 이미 load 주소(또는 PC 에서는 아무 버퍼)에 놓인 펌웨어의 벡터 표 검사. base 는 펌웨어가 돌 주소 */
enum app_check app_check_vectors(const uint8_t *image, uint32_t length, uint32_t base);

/* IEEE CRC-32 (zlib.crc32 와 같음). 이어서 계산하려면 앞 결과를 crc 로 넘긴다. 처음은 0 */
uint32_t app_crc32(uint32_t crc, const uint8_t *data, uint32_t length);

const char *app_check_name(enum app_check result);

#endif
