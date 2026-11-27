/* 기본 펌웨어(--app-link 로 빌드한 .bin)를 로더 이미지 안에 그대로 넣는다. 경로는 prepare_loader.py 가 쓴다 */
#include "loader_builtin.h"

__asm__(".section .rodata.loader_builtin,\"a\",%progbits\n"
        ".balign 8\n"
        ".global loader_builtin_start\n"
        "loader_builtin_start:\n"
        ".incbin \"" LOADER_BUILTIN_PATH "\"\n"
        ".global loader_builtin_end\n"
        "loader_builtin_end:\n"
        ".previous\n");
