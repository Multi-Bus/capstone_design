/* FAT32 카드에서 미리 만들어 둔 기록 파일의 위치를 찾는다 (읽기만 함, 하드웨어와 무관).
 *
 * 기록 펌웨어는 파일 시스템에 쓰지 않는다. PC 에서 큰 파일 하나(UWBLOG.BIN, 0 으로 채움)를 만들어 두면
 * 그 파일이 차지한 섹터 범위를 여기서 찾고, 펌웨어는 그 범위 안에 차례로 쓴다. 그래서 PC 에서 그대로 파일로 읽힌다.
 * 조건: 파일이 끊기지 않고 이어져 있어야 한다(갓 포맷한 카드에 처음 만든 파일은 그렇다). 아니면 오류를 낸다.
 */
#ifndef SDFAT_H
#define SDFAT_H

#include <stdint.h>

enum sdfat_result {
    SDFAT_OK = 0,
    SDFAT_READ_ERROR,      /* 섹터를 읽지 못함 */
    SDFAT_NOT_FAT32,       /* FAT32 가 아님 (exFAT 은 지원하지 않음. 32 GB 이하 카드를 FAT32 로 포맷) */
    SDFAT_NO_FILE,         /* 맨 위 폴더에 그 이름의 파일이 없음 */
    SDFAT_FRAGMENTED,      /* 파일이 여러 조각으로 나뉘어 있음: 카드를 포맷하고 파일을 다시 만들 것 */
    SDFAT_TOO_SMALL,       /* 파일이 한 섹터보다 작음 */
};

struct sdfat_file {
    uint32_t first_block;  /* 파일의 첫 섹터 (카드 전체에서의 번호) */
    uint32_t blocks;       /* 파일 크기 (섹터 수, 512바이트) */
    uint32_t cluster_blocks, partition_start, start_cluster;
    uint32_t root_entries; /* 맨 위 폴더에서 본 항목 수 (못 찾았을 때 참고) */
};

/* 한 섹터(512바이트)를 읽어 buffer 에 넣고 0 을 돌려주는 함수. 실패하면 0 이 아닌 값 */
typedef int (*sdfat_read_fn)(uint32_t block, uint8_t *buffer);

/* name83 은 "UWBLOG  BIN" 처럼 11글자(이름 8 + 확장자 3, 빈 자리는 공백, 대문자) */
enum sdfat_result sdfat_locate(sdfat_read_fn read, const char name83[11], struct sdfat_file *out);

/* 파일 안에서 아직 쓰지 않은 첫 섹터(첫 바이트가 0)를 찾는다. 쓴 부분이 앞에서부터 이어져 있다고 본다.
 * 결과는 0 ~ blocks (blocks 면 꽉 참). 읽기 오류면 0xFFFFFFFF */
uint32_t sdfat_find_end(sdfat_read_fn read, const struct sdfat_file *file);

const char *sdfat_result_name(enum sdfat_result result);

#endif
