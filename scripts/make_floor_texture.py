# 광류용 바닥 텍스처: 1024x1024 흑백 랜덤 칸(8px). 12.4m에 펼치면 칸 하나가 약 9.7cm.
import random, struct, zlib, sys
random.seed(42)  # 같은 결과를 다시 만들기 위해 고정
N, CELL = 1024, 8
vals = [[random.randint(40, 220) for _ in range(N // CELL)] for _ in range(N // CELL)]
raw = bytearray()
for y in range(N):
    raw.append(0)
    row = vals[y // CELL]
    raw.extend(bytes(row[x // CELL] for x in range(N)))
def chunk(tag, data):
    c = struct.pack(">I", len(data)) + tag + data
    return c + struct.pack(">I", zlib.crc32(tag + data) & 0xffffffff)
png = b"\x89PNG\r\n\x1a\n" + chunk(b"IHDR", struct.pack(">IIBBBBB", N, N, 8, 0, 0, 0, 0)) \
    + chunk(b"IDAT", zlib.compress(bytes(raw), 9)) + chunk(b"IEND", b"")
out = sys.argv[1]
open(out, "wb").write(png)
print("wrote", out, len(png), "bytes")
