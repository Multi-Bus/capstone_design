/* A* 경로 계획 (격자, 8방향). swarm/path_planning.py 의 포팅.
 * 작업 공간은 모두 정적 배열이다. heapq 대신 decrease-key 가 있는 이진 힙을 써서
 * 힙 크기가 칸 수를 넘지 않는다. 한 번에 한 태스크에서만 부른다 (재진입 불가). */
#include "swarm.h"

#include <math.h>

#define INF_COST ((real_t)1e30)
#define POS_NONE (-1)
#define POS_CLOSED (-2)

static const double SQRT2_D = 1.41421356237309504880;
static const struct { int dy, dx; real_t cost; } MOVES[8] = {
    {-1, 0, 1}, {1, 0, 1}, {0, -1, 1}, {0, 1, 1},
    {-1, -1, (real_t)1.41421356237309504880}, {-1, 1, (real_t)1.41421356237309504880},
    {1, -1, (real_t)1.41421356237309504880}, {1, 1, (real_t)1.41421356237309504880},
};

static real_t g_cost[SWARM_MAX_CELLS] SWARM_BIG;
static real_t g_key[SWARM_MAX_CELLS] SWARM_BIG;
static int32_t g_parent[SWARM_MAX_CELLS] SWARM_BIG;
static int32_t g_pos[SWARM_MAX_CELLS] SWARM_BIG;
static int32_t g_heap[SWARM_MAX_CELLS] SWARM_BIG;
static int g_hs;

static uint8_t g_inflated[SWARM_MAX_CELLS] SWARM_BIG;
static swarm_cell_t g_path[SWARM_MAX_CELLS] SWARM_BIG;

static void heap_swap(int a, int b)
{
    int32_t ia = g_heap[a], ib = g_heap[b];
    g_heap[a] = ib; g_heap[b] = ia;
    g_pos[ib] = a; g_pos[ia] = b;
}

static void sift_up(int i)
{
    while (i > 0) {
        int p = (i - 1) / 2;
        if (g_key[g_heap[i]] < g_key[g_heap[p]]) { heap_swap(i, p); i = p; } else break;
    }
}

static void sift_down(int i)
{
    for (;;) {
        int l = 2 * i + 1, r = l + 1, s = i;
        if (l < g_hs && g_key[g_heap[l]] < g_key[g_heap[s]]) s = l;
        if (r < g_hs && g_key[g_heap[r]] < g_key[g_heap[s]]) s = r;
        if (s == i) break;
        heap_swap(i, s);
        i = s;
    }
}

static void heap_push(int32_t idx)
{
    g_heap[g_hs] = idx;
    g_pos[idx] = g_hs;
    g_hs++;
    sift_up(g_hs - 1);
}

static int32_t heap_pop(void)
{
    int32_t top = g_heap[0];
    g_hs--;
    if (g_hs > 0) {
        g_heap[0] = g_heap[g_hs];
        g_pos[g_heap[0]] = 0;
        sift_down(0);
    }
    g_pos[top] = POS_CLOSED;
    return top;
}

int swarm_inflate(const uint8_t *grid, uint8_t *out, int h, int w, int r_cells)
{
    if (h <= 0 || w <= 0 || h * w > SWARM_MAX_CELLS) return SWARM_ERR_CAPACITY;
    for (int i = 0; i < h * w; i++) out[i] = grid[i];
    if (r_cells <= 0) return SWARM_OK;
    /* 줄마다 원 안쪽 구간(span)을 한 번에 칠한다 */
    int r2 = r_cells * r_cells;
    for (int y = 0; y < h; y++) {
        for (int x = 0; x < w; x++) {
            if (!grid[y * w + x]) continue;
            for (int dy = -r_cells; dy <= r_cells; dy++) {
                int yy = y + dy;
                if (yy < 0 || yy >= h) continue;
                int half = (int)floor(sqrt((double)(r2 - dy * dy)) + 1e-9);
                int x0 = x - half, x1 = x + half;
                if (x0 < 0) x0 = 0;
                if (x1 >= w) x1 = w - 1;
                for (int xx = x0; xx <= x1; xx++) out[yy * w + xx] = 1;
            }
        }
    }
    return SWARM_OK;
}

static real_t heuristic(int cy, int cx, int gy, int gx)
{
    int dy = cy > gy ? cy - gy : gy - cy;
    int dx = cx > gx ? cx - gx : gx - cx;
    int mn = dy < dx ? dy : dx;
    return (real_t)(dy + dx) + (real_t)(SQRT2_D - 2.0) * (real_t)mn; /* octile */
}

int swarm_astar(const uint8_t *grid, int h, int w, swarm_cell_t start, swarm_cell_t goal,
                swarm_cell_t *path, int cap, int *len)
{
    *len = 0;
    if (h <= 0 || w <= 0 || h * w > SWARM_MAX_CELLS) return SWARM_ERR_CAPACITY;
    if (start.y < 0 || start.y >= h || start.x < 0 || start.x >= w ||
        goal.y < 0 || goal.y >= h || goal.x < 0 || goal.x >= w) return SWARM_ERR_ARG;
    int32_t sidx = start.y * w + start.x, gidx = goal.y * w + goal.x;
    if (grid[sidx] || grid[gidx]) return SWARM_ERR_ARG;

    for (int i = 0; i < h * w; i++) { g_cost[i] = INF_COST; g_pos[i] = POS_NONE; g_parent[i] = -1; }
    g_hs = 0;
    g_cost[sidx] = 0;
    g_key[sidx] = heuristic(start.y, start.x, goal.y, goal.x);
    heap_push(sidx);

    while (g_hs > 0) {
        int32_t cur = heap_pop();
        if (cur == gidx) {
            int n = 0;
            for (int32_t c = cur; c >= 0; c = g_parent[c]) n++;
            if (n > cap) return SWARM_ERR_CAPACITY;
            int k = n;
            for (int32_t c = cur; c >= 0; c = g_parent[c]) {
                k--;
                path[k].y = (int16_t)(c / w);
                path[k].x = (int16_t)(c % w);
            }
            *len = n;
            return SWARM_OK;
        }
        int cy = cur / w, cx = cur % w;
        for (int m = 0; m < 8; m++) {
            int ny = cy + MOVES[m].dy, nx = cx + MOVES[m].dx;
            if (ny < 0 || ny >= h || nx < 0 || nx >= w) continue;
            int32_t nidx = ny * w + nx;
            if (grid[nidx] || g_pos[nidx] == POS_CLOSED) continue;
            if (MOVES[m].dy && MOVES[m].dx && (grid[(cy + MOVES[m].dy) * w + cx] || grid[cy * w + cx + MOVES[m].dx]))
                continue; /* 모서리 끼어들기 금지 */
            real_t ng = g_cost[cur] + MOVES[m].cost;
            if (ng < g_cost[nidx]) {
                g_cost[nidx] = ng;
                g_parent[nidx] = cur;
                g_key[nidx] = ng + heuristic(ny, nx, goal.y, goal.x);
                if (g_pos[nidx] >= 0) sift_up(g_pos[nidx]); else heap_push(nidx);
            }
        }
    }
    return SWARM_NO_PATH;
}

int swarm_line_free(const uint8_t *grid, int h, int w, swarm_cell_t a, swarm_cell_t b)
{
    int dy = b.y - a.y, dx = b.x - a.x;
    int adx = dx < 0 ? -dx : dx, ady = dy < 0 ? -dy : dy;
    int n = (adx > ady ? adx : ady) * 2 + 1;
    double step = 1.0 / n;
    for (int k = 0; k <= n; k++) {
        double t = (k == n) ? 1.0 : k * step;          /* numpy.linspace 와 같은 표본 */
        int y = (int)nearbyint(a.y + dy * t);           /* Python round 와 같은 반올림(짝수 쪽) */
        int x = (int)nearbyint(a.x + dx * t);
        if (y < 0 || y >= h || x < 0 || x >= w || grid[y * w + x]) return 0;
    }
    return 1;
}

int swarm_shortcut(const uint8_t *grid, int h, int w, swarm_cell_t *path, int n)
{
    if (n <= 0) return n;
    int wr = 0, i = 0;
    while (i < n - 1) {
        int j = n - 1;
        while (j > i + 1 && !swarm_line_free(grid, h, w, path[i], path[j])) j--;
        swarm_cell_t c = path[j];
        path[++wr] = c;
        i = j;
    }
    return wr + 1;
}

static int to_cell(real_t v, double res)
{
    return (int)nearbyint((double)v / res);
}

int swarm_plan(const uint8_t *grid, int h, int w, real_t res, vec2_t start_xy, vec2_t goal_xy,
               real_t inflate_m, vec2_t *out, int cap, int *len)
{
    *len = 0;
    const uint8_t *g = grid;
    if (inflate_m > 0) {
        int r = (int)ceil((double)inflate_m / (double)res);
        int rc = swarm_inflate(grid, g_inflated, h, w, r);
        if (rc != SWARM_OK) return rc;
        g = g_inflated;
    }
    double r = (double)res;
    swarm_cell_t s = { (int16_t)to_cell(start_xy.y, r), (int16_t)to_cell(start_xy.x, r) };
    swarm_cell_t t = { (int16_t)to_cell(goal_xy.y, r), (int16_t)to_cell(goal_xy.x, r) };
    int n = 0;
    int rc = swarm_astar(g, h, w, s, t, g_path, SWARM_MAX_CELLS, &n);
    if (rc != SWARM_OK) return rc;
    n = swarm_shortcut(g, h, w, g_path, n);
    if (n > cap) return SWARM_ERR_CAPACITY;
    for (int i = 0; i < n; i++) {
        out[i].x = (real_t)(g_path[i].x * r);
        out[i].y = (real_t)(g_path[i].y * r);
    }
    *len = n;
    return SWARM_OK;
}
