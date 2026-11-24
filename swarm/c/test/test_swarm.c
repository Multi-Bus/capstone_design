/* swarm C 포팅 점검. 빌드와 실행은 swarm/c/README.md.
 * 마지막에 이 PC 에서 잰 시간을 찍는다. N6 시간이 아니다. N6 에서는 Task 3 Lv3 로 따로 잰다. */
#include "swarm.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

static int fails;
#define CHECK(c) do { if (!(c)) { printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #c); fails++; } } while (0)
#define NEAR(a, b, tol) CHECK(fabs((double)(a) - (double)(b)) <= (tol))

static uint8_t grid[SWARM_MAX_CELLS];

static void test_qp(void)
{
    vec2_t out;
    /* 제약이 없거나 이미 만족: v_des 그대로 */
    CHECK(swarm_solve_qp((vec2_t){1, 0}, NULL, NULL, 0, &out) == SWARM_OK);
    NEAR(out.x, 1, 1e-6); NEAR(out.y, 0, 1e-6);
    /* x >= 2 제약: v_des (1, 0.5) 는 (2, 0.5) 로 사영 */
    vec2_t A1[1] = {{1, 0}};
    real_t b1[1] = {2};
    CHECK(swarm_solve_qp((vec2_t){1, (real_t)0.5}, A1, b1, 1, &out) == SWARM_OK);
    NEAR(out.x, 2, 1e-5); NEAR(out.y, 0.5, 1e-5);
    /* x >= 2 와 x <= 1 (= -x >= -1) 은 모순: 정지 */
    vec2_t A2[2] = {{1, 0}, {-1, 0}};
    real_t b2[2] = {2, -1};
    CHECK(swarm_solve_qp((vec2_t){0, 0}, A2, b2, 2, &out) == SWARM_INFEASIBLE);
    NEAR(out.x, 0, 0); NEAR(out.y, 0, 0);
}

static void test_cbf(void)
{
    vec2_t out;
    /* 정면에서 마주 오는 이웃이 없으면 원하는 속도 그대로 (상자 제약 안쪽) */
    CHECK(swarm_cbf_qp((vec2_t){(real_t)0.5, 0}, (vec2_t){0, 0}, NULL, 0, (real_t)0.4, 1, 1, &out) == SWARM_OK);
    NEAR(out.x, 0.5, 1e-6);
    /* 0.3 m 앞에 정지한 이웃(D = 0.4 안쪽): 전진 속도를 줄여야 한다 */
    swarm_nbr_t nb = {{(real_t)0.3, 0}, {0, 0}, 0};
    CHECK(swarm_cbf_qp((vec2_t){1, 0}, (vec2_t){0, 0}, &nb, 1, (real_t)0.4, 1, 1, &out) == SWARM_OK);
    CHECK(out.x < (real_t)0.5);
    /* 제약이 상한을 넘으면 거부 */
    swarm_nbr_t many[SWARM_MAX_ROWS];
    memset(many, 0, sizeof many);
    CHECK(swarm_cbf_qp((vec2_t){0, 0}, (vec2_t){0, 0}, many, SWARM_MAX_ROWS, 1, 1, 1, &out) == SWARM_ERR_CAPACITY);
}

static void test_misc(void)
{
    /* 편대: 이웃과 자리가 맞고 목표 위에 있으면 속도 0 */
    swarm_fnbr_t nb = {{1, 0}, {1, 0}};
    vec2_t u = swarm_formation_vel((vec2_t){0, 0}, (vec2_t){0, 0}, &nb, 1, (vec2_t){0, 0}, (real_t)0.5, (real_t)0.6, 1);
    NEAR(u.x, 0, 1e-6); NEAR(u.y, 0, 1e-6);
    /* 크기 제한 */
    u = swarm_formation_vel((vec2_t){0, 0}, (vec2_t){0, 0}, NULL, 0, (vec2_t){100, 0}, (real_t)0.5, (real_t)0.6, 1);
    NEAR(u.x, 1, 1e-5);
    /* 웨이포인트 추종 */
    vec2_t wp[3] = {{0, 0}, {1, 0}, {2, 0}};
    swarm_follower_t f;
    swarm_follower_init(&f, wp, 3, (real_t)0.3);
    vec2_t r = swarm_follower_ref(&f, (vec2_t){0, 0});
    NEAR(r.x, 2 - 1, 1e-6);                 /* 시작점 반경 안이라 다음 점으로 넘어감 */
    CHECK(!swarm_follower_done(&f, (vec2_t){0, 0}));
    r = swarm_follower_ref(&f, (vec2_t){1, 0});
    NEAR(r.x, 2, 1e-6);
    CHECK(swarm_follower_done(&f, (vec2_t){(real_t)1.9, 0}));
    /* 이벤트 트리거: 처음에는 보낸다, 등속 외삽과 같게 움직이면 안 보낸다, 벗어나면 보낸다, max_gap 지나면 보낸다 */
    swarm_trigger_t e;
    swarm_trigger_init(&e, (real_t)0.1, 1);
    CHECK(swarm_trigger_should_send(&e, (vec2_t){0, 0}, (vec2_t){1, 0}, 0, 0));
    swarm_trigger_sent(&e, (vec2_t){0, 0}, (vec2_t){1, 0}, 0);
    CHECK(!swarm_trigger_should_send(&e, (vec2_t){(real_t)0.5, 0}, (vec2_t){1, 0}, (real_t)0.5, 0));
    CHECK(swarm_trigger_should_send(&e, (vec2_t){(real_t)0.5, (real_t)0.3}, (vec2_t){1, 0}, (real_t)0.5, 0));
    CHECK(swarm_trigger_should_send(&e, (vec2_t){1, 0}, (vec2_t){1, 0}, 1, 0));
    NEAR(swarm_effective_distance((real_t)0.4, (real_t)0.1), 0.6, 1e-6);
    /* 교착: 목표에서 멀고 명령이 0 인 상태가 hold 초 이어지면 60도 돌린 속도를 낸다 */
    swarm_deadlock_t d;
    swarm_deadlock_init(&d, 1, (real_t)1.5, 60, (real_t)0.05, (real_t)0.5);
    vec2_t v = {1, 0}, z = {0, 0};
    for (int k = 0; k <= 10; k++) v = swarm_deadlock_adjust(&d, (vec2_t){1, 0}, z, 5, k * (real_t)0.1, (real_t)0.1);
    CHECK(d.count == 1);
    NEAR(v.x, 0.5, 1e-5); NEAR(v.y, -0.8660254, 1e-5);
}

static void make_grid(int h, int w) { memset(grid, 0, (size_t)h * w); }

static void test_astar(void)
{
    int h = 21, w = 21, n;
    make_grid(h, w);
    swarm_cell_t path[SWARM_MAX_CELLS];
    /* 빈 격자: 대각선 10칸 + 직선 */
    CHECK(swarm_astar(grid, h, w, (swarm_cell_t){0, 0}, (swarm_cell_t){10, 20}, path, SWARM_MAX_CELLS, &n) == SWARM_OK);
    CHECK(path[0].y == 0 && path[0].x == 0 && path[n - 1].y == 10 && path[n - 1].x == 20);
    CHECK(n == 21);                                   /* 대각 10 + 직선 10 + 시작 1 */
    /* 벽(열 10)에 틈 하나 (행 18): 틈을 지나가야 한다 */
    for (int y = 0; y < h; y++) grid[y * w + 10] = 1;
    grid[18 * w + 10] = 0;
    CHECK(swarm_astar(grid, h, w, (swarm_cell_t){0, 0}, (swarm_cell_t){0, 20}, path, SWARM_MAX_CELLS, &n) == SWARM_OK);
    int through = 0;
    for (int i = 0; i < n; i++) if (path[i].x == 10 && path[i].y == 18) through = 1;
    CHECK(through);
    /* 틈을 막으면 길 없음 */
    grid[18 * w + 10] = 1;
    CHECK(swarm_astar(grid, h, w, (swarm_cell_t){0, 0}, (swarm_cell_t){0, 20}, path, SWARM_MAX_CELLS, &n) == SWARM_NO_PATH);
    /* 장애물 위 시작, 범위 밖 */
    CHECK(swarm_astar(grid, h, w, (swarm_cell_t){0, 10}, (swarm_cell_t){0, 20}, path, SWARM_MAX_CELLS, &n) == SWARM_ERR_ARG);
    CHECK(swarm_astar(grid, h, w, (swarm_cell_t){0, 0}, (swarm_cell_t){0, 99}, path, SWARM_MAX_CELLS, &n) == SWARM_ERR_ARG);
    /* 경로 용량이 모자라면 오류 */
    make_grid(h, w);
    CHECK(swarm_astar(grid, h, w, (swarm_cell_t){0, 0}, (swarm_cell_t){0, 20}, path, 5, &n) == SWARM_ERR_CAPACITY);
}

static void test_plan(void)
{
    /* 15 m / 0.1 m 해상도 방, 가운데 반지름 1 m 기둥. 시작 (2, 7.5) → 목표 (13, 7.5) */
    int h = 151, w = 151;
    for (int y = 0; y < h; y++)
        for (int x = 0; x < w; x++) {
            double dx = x * 0.1 - 7.5, dy = y * 0.1 - 7.5;
            grid[y * w + x] = (dx * dx + dy * dy <= 1.0) ? 1 : 0;
        }
    vec2_t out[SWARM_MAX_CELLS];
    int n;
    CHECK(swarm_plan(grid, h, w, (real_t)0.1, (vec2_t){2, (real_t)7.5}, (vec2_t){13, (real_t)7.5}, (real_t)1.2, out, SWARM_MAX_CELLS, &n) == SWARM_OK);
    CHECK(n >= 3);
    NEAR(out[0].x, 2, 1e-4); NEAR(out[n - 1].x, 13, 1e-4);
    for (int i = 0; i < n; i++) {                      /* 모든 웨이포인트가 기둥에서 1.2 m 밖 */
        double dx = out[i].x - 7.5, dy = out[i].y - 7.5;
        CHECK(sqrt(dx * dx + dy * dy) >= 1.2 - 0.06);
    }
}

static double now_us(void)
{
    return (double)clock() * 1e6 / CLOCKS_PER_SEC;
}

static void bench(void)
{
    /* CBF-QP: 이웃 5 + 기둥 3 (제약 8 + 상자 4 = 12) */
    swarm_nbr_t nb[8];
    for (int i = 0; i < 8; i++) nb[i] = (swarm_nbr_t){{(real_t)(0.3 + 0.05 * i), (real_t)(0.2 - 0.06 * i)}, {0, 0}, 0};
    vec2_t out;
    const int N = 20000;
    double t0 = now_us();
    for (int i = 0; i < N; i++) swarm_cbf_qp((vec2_t){1, (real_t)(0.001 * (i & 7))}, (vec2_t){0, 0}, nb, 8, (real_t)0.4, 1, 1, &out);
    printf("CBF-QP (제약 12): %.2f us/회 (이 PC)\n", (now_us() - t0) / N);

    /* A*: 151 x 151, 장애물 없이 대각선 + 가운데 기둥 */
    int h = 151, w = 151;
    for (int y = 0; y < h; y++)
        for (int x = 0; x < w; x++) {
            double dx = x * 0.1 - 7.5, dy = y * 0.1 - 7.5;
            grid[y * w + x] = (dx * dx + dy * dy <= 1.0) ? 1 : 0;
        }
    static vec2_t wp[SWARM_MAX_CELLS];
    int n;
    const int M = 20;
    t0 = now_us();
    for (int i = 0; i < M; i++) swarm_plan(grid, h, w, (real_t)0.1, (vec2_t){(real_t)0.5, (real_t)0.5}, (vec2_t){14, 14}, (real_t)1.2, wp, SWARM_MAX_CELLS, &n);
    printf("plan (A* + 부풀리기 + 지름길, 151x151): %.2f ms/회 (이 PC)\n", (now_us() - t0) / M / 1000.0);
}

int main(void)
{
    test_qp();
    test_cbf();
    test_misc();
    test_astar();
    test_plan();
    if (fails) { printf("%d FAILED\n", fails); return 1; }
    printf("ALL PASS (real_t = %s)\n", sizeof(real_t) == 4 ? "float" : "double");
    bench();
    return 0;
}
