/* Host driver for link_failsafe.h. stdin lines:
 *   G <ms> <hex>   bytes laptop->FC at time ms
 *   F <ms> <hex>   bytes FC->laptop at time ms
 *   P <ms>         poll; prints "OUT <hex>" or "NONE"                         */
#include <stdio.h>
#include "../FSBL/Inc/link_failsafe.h"

static LinkFs_t fs;

int main(void)
{
  char kind; unsigned long t; char hex[4096];
  while (scanf(" %c %lu", &kind, &t) == 2)
  {
    if (kind == 'P')
    {
      uint8_t out[LINKFS_FRAME_LEN];
      uint32_t n = LinkFs_Poll(&fs, (uint32_t)t, out), i;
      if (n == 0) { puts("NONE"); continue; }
      printf("OUT ");
      for (i = 0; i < n; i++) { printf("%02x", out[i]); }
      puts("");
      continue;
    }
    if (scanf("%4095s", hex) != 1) { return 1; }
    for (char *h = hex; h[0] && h[1]; h += 2)
    {
      unsigned v; sscanf(h, "%2x", &v);
      if (kind == 'G') { LinkFs_OnGcsByte(&fs, (uint8_t)v, (uint32_t)t); }
      else             { LinkFs_OnFcByte(&fs, (uint8_t)v, (uint32_t)t); }
    }
  }
  return 0;
}
