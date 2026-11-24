/* swarm.h - 편대 제어, 충돌 회피, 경로 계획 (swarm 폴더 파이썬 파일들의 C99 포팅)
 *
 * 대상: STM32N657 (Cortex-M55). malloc 없음, 재귀 없음, 반복 횟수 상한이 정해진 코드다.
 * real_t 기본은 float (M55 FPU). -DSWARM_DOUBLE 로 double 로 빌드하면 Python 과 같은 정밀도로 비교할 수 있다.
 * 2차원(x, y)만 다룬다. Python solve_qp 는 d = 2 ~ 3 을 받지만 기체 제어는 2차원이다.
 */
#ifndef SWARM_H
#define SWARM_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#ifdef SWARM_DOUBLE
typedef double real_t;
#else
typedef float real_t;
#endif

typedef struct { real_t x, y; } vec2_t;

enum {
    SWARM_OK = 0,
    SWARM_INFEASIBLE = 1,   /* QP 해 없음 (정지 명령을 돌려준다) */
    SWARM_ERR_CAPACITY = 2, /* 제약 · 경로 · 격자가 컴파일 시 상한을 넘음 */
    SWARM_NO_PATH = 3,      /* A* 가 길을 못 찾음 */
    SWARM_ERR_ARG = 4       /* 범위 밖 칸, 시작이나 목표가 장애물 */
};

/* ---- 충돌 회피 (collision_avoidance.py) ---- */

#ifndef SWARM_MAX_ROWS
#define SWARM_MAX_ROWS 32   /* 이웃 + 장애물 + 상자 제약 4개 */
#endif

typedef struct {
    vec2_t p, v;
    real_t d;   /* CBF: 이 이웃의 안전거리. d <= 0 이면 기본 D 를 쓴다. APF: 표면까지 거리를 재는 반경 (0 이면 중심) */
} swarm_nbr_t;

/* min |v - v_des|^2  s.t.  A v >= b. 활성 제약 조합 전수 확인이라 반복이 없고 시간이 일정하다 (O(m^2)).
 * 해가 없으면 out = 0, SWARM_INFEASIBLE. m > SWARM_MAX_ROWS 면 SWARM_ERR_CAPACITY. */
int swarm_solve_qp(vec2_t v_des, const vec2_t *A, const real_t *b, int m, vec2_t *out);

/* CBF 제약 (이웃 하나당 한 줄) + 상자 제약 |v_x|, |v_y| <= vmax 로 QP 를 푼다. */
int swarm_cbf_qp(vec2_t v_des, vec2_t p_i, const swarm_nbr_t *others, int n,
                 real_t D, real_t gamma, real_t vmax, vec2_t *out);

vec2_t swarm_apf(vec2_t v_des, vec2_t p_i, const swarm_nbr_t *others, int n,
                 real_t rho0, real_t k_rep, real_t vmax);

typedef struct {
    real_t hold, escape, v_min, far;
    real_t R[4];          /* 오른쪽으로 angle 만큼 돌리는 행렬 (행 우선) */
    real_t stall, until;
    int count;
} swarm_deadlock_t;

/* Python 기본값: hold 1.0, escape 1.5, angle_deg 60, v_min 0.05, far 0.5 */
void swarm_deadlock_init(swarm_deadlock_t *d, real_t hold, real_t escape, real_t angle_deg,
                         real_t v_min, real_t far_);
vec2_t swarm_deadlock_adjust(swarm_deadlock_t *d, vec2_t v_des, vec2_t u_prev,
                             real_t dist_goal, real_t t, real_t dt);

real_t swarm_safe_distance(real_t base, real_t eps_self, real_t eps_nbr, real_t v_rel_max,
                           real_t age, real_t clock_err);

/* ---- 편대 제어 (formation_control.py) ---- */

typedef struct { vec2_t p, d; } swarm_fnbr_t;   /* 이웃 예측 위치, 이웃의 대형 오프셋 */

/* u_i = k_f * sum_j (p_j - p_i - (d_j - d_i)) + k_g * (c_ref + d_i - p_i), 크기를 vmax 로 제한 */
vec2_t swarm_formation_vel(vec2_t p_i, vec2_t d_i, const swarm_fnbr_t *nbrs, int n,
                           vec2_t c_ref, real_t k_f, real_t k_g, real_t vmax);

typedef struct { const vec2_t *wp; int n; int i; real_t reach; } swarm_follower_t;
void swarm_follower_init(swarm_follower_t *f, const vec2_t *wp, int n, real_t reach); /* n >= 1 */
vec2_t swarm_follower_ref(swarm_follower_t *f, vec2_t center);
int swarm_follower_done(const swarm_follower_t *f, vec2_t center);

/* ---- 이벤트 트리거 (event_trigger.py) ---- */

typedef struct {
    real_t delta, max_gap;
    int has_last;
    vec2_t p, v;
    real_t t;
} swarm_trigger_t;

void swarm_trigger_init(swarm_trigger_t *e, real_t delta, real_t max_gap); /* Python 기본 max_gap 1.0 */
vec2_t swarm_predict(vec2_t p, vec2_t v, real_t t_src, real_t t);
int swarm_trigger_should_send(const swarm_trigger_t *e, vec2_t p, vec2_t v, real_t t, int path_changed);
void swarm_trigger_sent(swarm_trigger_t *e, vec2_t p, vec2_t v, real_t t);
real_t swarm_effective_distance(real_t D, real_t delta);

/* ---- 경로 계획 (path_planning.py) ---- */

#ifndef SWARM_MAX_CELLS
#define SWARM_MAX_CELLS 22801   /* 격자 칸 수 상한. 15 m / 0.1 m 해상도 = 151 x 151 */
#endif

typedef struct { int16_t y, x; } swarm_cell_t;   /* (행, 열) */

/* A* 작업 공간(약 0.5 MB)을 둘 링커 구역. 기본은 일반 .bss.
 * N6 FSBL 은 RAM 이 256 KB 라서 SWARM_USE_AXISRAM1 을 정의해 .axisram1 구역(AXISRAM1)에 둔다. */
#ifndef SWARM_BIG
#  if defined(SWARM_USE_AXISRAM1)
#    define SWARM_BIG __attribute__((section(".axisram1")))
#  else
#    define SWARM_BIG
#  endif
#endif

/* 격자: uint8 배열, 1 = 장애물, 행 우선 (index = y * w + x). out 은 grid 와 같은 크기. */
int swarm_inflate(const uint8_t *grid, uint8_t *out, int h, int w, int r_cells);

/* 8방향 A*. 모서리 끼어들기 금지. path 에 시작 → 목표 칸을 채우고 길이를 len 에 넣는다. */
int swarm_astar(const uint8_t *grid, int h, int w, swarm_cell_t start, swarm_cell_t goal,
                swarm_cell_t *path, int cap, int *len);

int swarm_line_free(const uint8_t *grid, int h, int w, swarm_cell_t a, swarm_cell_t b);

/* 막히지 않는 웨이포인트를 건너뛴다. path 를 제자리에서 줄이고 새 길이를 돌려준다. */
int swarm_shortcut(const uint8_t *grid, int h, int w, swarm_cell_t *path, int n);

/* m 좌표 (x, y) 시작점에서 목표점까지 웨이포인트. inflate_m > 0 이면 장애물을 부풀린다.
 * 내부에 정적 작업 공간을 쓰므로 한 번에 한 태스크에서만 부른다 (재진입 불가). */
int swarm_plan(const uint8_t *grid, int h, int w, real_t res, vec2_t start_xy, vec2_t goal_xy,
               real_t inflate_m, vec2_t *out, int cap, int *len);

#ifdef __cplusplus
}
#endif
#endif /* SWARM_H */
