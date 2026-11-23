/*
 * newlib 이 요구하는 최소 시스템 호출. 파일 입출력은 쓰지 않으므로 모두 실패를 돌려준다.
 * (nosys.specs 를 쓰면 같은 일을 하지만 링크 때 경고가 나와서 직접 둔다.)
 * 힙은 gcc_startup_nrf52833.S 의 __HeapBase ~ __HeapLimit (8 KB) 를 쓴다.
 */
#include <errno.h>
#include <stddef.h>
#include <stdint.h>
#include <sys/stat.h>

extern char __HeapBase;
extern char __HeapLimit;

void *_sbrk(ptrdiff_t increment)
{
    static char *heap_end = &__HeapBase;
    char *previous = heap_end;
    if (heap_end + increment > &__HeapLimit) {
        errno = ENOMEM;
        return (void *)-1;
    }
    heap_end += increment;
    return previous;
}

int _close(int file) { (void)file; return -1; }
int _fstat(int file, struct stat *st) { (void)file; st->st_mode = S_IFCHR; return 0; }
int _isatty(int file) { (void)file; return 1; }
int _lseek(int file, int offset, int whence) { (void)file; (void)offset; (void)whence; return 0; }
int _read(int file, char *buffer, int length) { (void)file; (void)buffer; (void)length; return 0; }
int _write(int file, const char *buffer, int length) { (void)file; (void)buffer; return length; }
int _getpid(void) { return 1; }
int _kill(int pid, int signal) { (void)pid; (void)signal; errno = EINVAL; return -1; }
void _exit(int status) { (void)status; for (;;) { } }
