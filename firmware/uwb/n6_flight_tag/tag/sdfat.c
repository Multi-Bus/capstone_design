#include <string.h>
#include "sdfat.h"

enum { SECTOR = 512, ENTRY = 32, END_OF_CHAIN = 0x0FFFFFF8UL };

static uint16_t get16(const uint8_t *p) { return (uint16_t)(p[0] | (p[1] << 8)); }
static uint32_t get32(const uint8_t *p)
{
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

struct volume {
    sdfat_read_fn read;
    uint32_t fat_start, data_start, cluster_blocks, clusters;
    uint32_t cached;          /* fat 버퍼에 들어 있는 FAT 섹터 번호 */
    uint8_t fat[SECTOR];
};

/* 클러스터 cluster 다음 클러스터. 읽기 오류면 0 */
static uint32_t next_cluster(struct volume *v, uint32_t cluster)
{
    const uint32_t sector = v->fat_start + cluster / (SECTOR / 4);
    if (sector != v->cached) {
        if (v->read(sector, v->fat) != 0) {
            v->cached = 0;
            return 0;
        }
        v->cached = sector;
    }
    return get32(v->fat + 4 * (cluster % (SECTOR / 4))) & 0x0FFFFFFFUL;
}

static int is_fat32_boot(const uint8_t *s)
{
    return s[510] == 0x55 && s[511] == 0xAA && get16(s + 11) == SECTOR && s[13] != 0 && get16(s + 17) == 0 &&
           get32(s + 36) != 0 && memcmp(s + 82, "FAT32   ", 8) == 0;
}

enum sdfat_result sdfat_locate(sdfat_read_fn read, const char name83[11], struct sdfat_file *out)
{
    static struct volume v;       /* 스택을 아끼려고 정적으로 둔다. 부팅 때 한 번만 부른다 */
    static uint8_t sector[SECTOR];
    uint32_t partition = 0;

    memset(out, 0, sizeof(*out));
    memset(&v, 0, sizeof(v));
    v.read = read;
    if (read(0, sector) != 0) {
        return SDFAT_READ_ERROR;
    }
    if (!is_fat32_boot(sector)) {
        /* 0번 섹터가 파티션 표인 경우: 첫 파티션을 본다 */
        if (sector[510] != 0x55 || sector[511] != 0xAA) {
            return SDFAT_NOT_FAT32;
        }
        partition = get32(sector + 446 + 8);
        if (partition == 0 || read(partition, sector) != 0) {
            return partition == 0 ? SDFAT_NOT_FAT32 : SDFAT_READ_ERROR;
        }
        if (!is_fat32_boot(sector)) {
            return SDFAT_NOT_FAT32;
        }
    }
    const uint32_t reserved = get16(sector + 14), fats = sector[16], fat_size = get32(sector + 36);
    const uint32_t total = get16(sector + 19) ? get16(sector + 19) : get32(sector + 32);
    uint32_t cluster = get32(sector + 44); /* 맨 위 폴더의 첫 클러스터 */
    v.cluster_blocks = sector[13];
    v.fat_start = partition + reserved;
    v.data_start = v.fat_start + fats * fat_size;
    v.clusters = (total - reserved - fats * fat_size) / v.cluster_blocks;
    out->partition_start = partition;
    out->cluster_blocks = v.cluster_blocks;

    /* 맨 위 폴더를 훑는다 */
    uint32_t start = 0, size = 0;
    int found = 0, end = 0;
    for (uint32_t guard = 0; !found && !end && cluster >= 2 && cluster < END_OF_CHAIN && guard < 65536; ++guard) {
        for (uint32_t s = 0; s < v.cluster_blocks && !found && !end; ++s) {
            if (read(v.data_start + (cluster - 2) * v.cluster_blocks + s, sector) != 0) {
                return SDFAT_READ_ERROR;
            }
            for (unsigned i = 0; i < SECTOR; i += ENTRY) {
                const uint8_t *e = sector + i;
                if (e[0] == 0x00) {
                    end = 1;
                    break;
                }
                if (e[0] == 0xE5 || (e[11] & 0x0F) == 0x0F || (e[11] & 0x18) != 0) {
                    continue; /* 지운 항목, 긴 이름 조각, 폴더, 볼륨 이름 */
                }
                ++out->root_entries;
                if (memcmp(e, name83, 11) == 0) {
                    start = ((uint32_t)get16(e + 20) << 16) | get16(e + 26);
                    size = get32(e + 28);
                    found = 1;
                    break;
                }
            }
        }
        if (!found && !end) {
            cluster = next_cluster(&v, cluster);
            if (cluster == 0) {
                return SDFAT_READ_ERROR;
            }
        }
    }
    if (!found) {
        return SDFAT_NO_FILE;
    }
    if (size < SECTOR || start < 2) {
        return SDFAT_TOO_SMALL;
    }

    /* 파일이 한 덩어리로 이어져 있는지: 클러스터 번호가 1씩 늘어야 한다 */
    const uint32_t bytes_per_cluster = v.cluster_blocks * SECTOR;
    const uint32_t count = (size + bytes_per_cluster - 1) / bytes_per_cluster;
    cluster = start;
    for (uint32_t i = 1; i < count; ++i) {
        const uint32_t next = next_cluster(&v, cluster);
        if (next == 0) {
            return SDFAT_READ_ERROR;
        }
        if (next != cluster + 1) {
            return SDFAT_FRAGMENTED;
        }
        cluster = next;
    }
    out->start_cluster = start;
    out->first_block = v.data_start + (start - 2) * v.cluster_blocks;
    out->blocks = size / SECTOR;
    return SDFAT_OK;
}

uint32_t sdfat_find_end(sdfat_read_fn read, const struct sdfat_file *file)
{
    static uint8_t sector[SECTOR];
    uint32_t lo = 0, hi = file->blocks; /* [0, lo) 는 쓴 섹터, [hi, blocks) 는 안 쓴 섹터 */
    while (lo < hi) {
        const uint32_t mid = lo + (hi - lo) / 2;
        if (read(file->first_block + mid, sector) != 0) {
            return 0xFFFFFFFFUL;
        }
        if (sector[0] != 0) {
            lo = mid + 1;
        } else {
            hi = mid;
        }
    }
    return lo;
}

const char *sdfat_result_name(enum sdfat_result result)
{
    switch (result) {
    case SDFAT_OK: return "ok";
    case SDFAT_READ_ERROR: return "read_error";
    case SDFAT_NOT_FAT32: return "not_fat32";
    case SDFAT_NO_FILE: return "no_file";
    case SDFAT_FRAGMENTED: return "fragmented";
    case SDFAT_TOO_SMALL: return "too_small";
    default: return "unknown";
    }
}
