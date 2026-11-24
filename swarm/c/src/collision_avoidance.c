/* 순간 회피: CBF-QP, APF, 교착 완화. swarm/collision_avoidance.py 의 포팅. */
#include "swarm.h"

#include <math.h>

#ifdef SWARM_DOUBLE
#define SQRT sqrt
#define COS cos
#define SIN sin
#define FABS fabs
#define DET_TOL 1e-12     /* Python 과 같은 값 */
#define FEAS_TOL 1e-7
#define SKIP_TOL 1e-9
#else
#define SQRT sqrtf
#define COS cosf
#define SIN sinf
#define FABS fabsf
#define DET_TOL 1e-9f     /* float 는 정밀도가 낮아 허용치를 넓힌다 */
#define FEAS_TOL 1e-5f
#define SKIP_TOL 1e-6f
#endif

static real_t dot(vec2_t a, vec2_t b) { return a.x * b.x + a.y * b.y; }

int swarm_solve_qp(vec2_t v_des, const vec2_t *A, const real_t *b, int m, vec2_t *out)
{
    if (m > SWARM_MAX_ROWS) {
        out->x = out->y = 0;
        return SWARM_ERR_CAPACITY;
    }
    int free_ok = 1;
    for (int i = 0; i < m; i++) {
        if (dot(A[i], v_des) < b[i] - SKIP_TOL) { free_ok = 0; break; }
    }
    if (free_ok) { *out = v_des; return SWARM_OK; }

    int found = 0;
    real_t best_dist = 0;
    vec2_t best = {0, 0};

#define CHECK_CANDIDATE(vc)                                              \
    do {                                                                 \
        int ok_ = 1;                                                     \
        for (int r_ = 0; r_ < m; r_++) {                                 \
            if (dot(A[r_], (vc)) < b[r_] - FEAS_TOL) { ok_ = 0; break; } \
        }                                                                \
        if (ok_) {                                                       \
            real_t dx_ = (vc).x - v_des.x, dy_ = (vc).y - v_des.y;       \
            real_t dist_ = dx_ * dx_ + dy_ * dy_;                        \
            if (!found || dist_ < best_dist) { found = 1; best_dist = dist_; best = (vc); } \
        }                                                                \
    } while (0)

    /* 활성 제약 1개: v_des 를 그 제약의 경계선에 사영 */
    for (int i = 0; i < m; i++) {
        real_t M = dot(A[i], A[i]);
        if (FABS(M) < DET_TOL) continue;
        real_t lam = (b[i] - dot(A[i], v_des)) / M;
        vec2_t v = { v_des.x + A[i].x * lam, v_des.y + A[i].y * lam };
        CHECK_CANDIDATE(v);
    }
    /* 활성 제약 2개: 두 경계선의 교점 (2차원이므로 v 가 하나로 정해진다) */
    for (int i = 0; i < m; i++) {
        for (int j = i + 1; j < m; j++) {
            real_t m00 = dot(A[i], A[i]), m01 = dot(A[i], A[j]), m11 = dot(A[j], A[j]);
            real_t det = m00 * m11 - m01 * m01;
            if (FABS(det) < DET_TOL) continue;
            real_t r0 = b[i] - dot(A[i], v_des), r1 = b[j] - dot(A[j], v_des);
            real_t l0 = (m11 * r0 - m01 * r1) / det;
            real_t l1 = (-m01 * r0 + m00 * r1) / det;
            vec2_t v = { v_des.x + A[i].x * l0 + A[j].x * l1, v_des.y + A[i].y * l0 + A[j].y * l1 };
            CHECK_CANDIDATE(v);
        }
    }
#undef CHECK_CANDIDATE

    if (!found) {
        out->x = out->y = 0;
        return SWARM_INFEASIBLE;
    }
    *out = best;
    return SWARM_OK;
}

int swarm_cbf_qp(vec2_t v_des, vec2_t p_i, const swarm_nbr_t *others, int n,
                 real_t D, real_t gamma, real_t vmax, vec2_t *out)
{
    vec2_t A[SWARM_MAX_ROWS];
    real_t b[SWARM_MAX_ROWS];
    if (n < 0 || n + 4 > SWARM_MAX_ROWS) {
        out->x = out->y = 0;
        return SWARM_ERR_CAPACITY;
    }
    for (int k = 0; k < n; k++) {
        real_t Dj = others[k].d > 0 ? others[k].d : D;
        vec2_t nrm = { p_i.x - others[k].p.x, p_i.y - others[k].p.y };
        real_t h = dot(nrm, nrm) - Dj * Dj;
        A[k] = nrm;
        b[k] = dot(nrm, others[k].v) - (real_t)0.5 * gamma * h;
    }
    int m = n;
    A[m] = (vec2_t){1, 0};  b[m++] = -vmax;
    A[m] = (vec2_t){-1, 0}; b[m++] = -vmax;
    A[m] = (vec2_t){0, 1};  b[m++] = -vmax;
    A[m] = (vec2_t){0, -1}; b[m++] = -vmax;
    return swarm_solve_qp(v_des, A, b, m, out);
}

vec2_t swarm_apf(vec2_t v_des, vec2_t p_i, const swarm_nbr_t *others, int n,
                 real_t rho0, real_t k_rep, real_t vmax)
{
    vec2_t v = v_des;
    for (int k = 0; k < n; k++) {
        vec2_t nrm = { p_i.x - others[k].p.x, p_i.y - others[k].p.y };
        real_t dist = SQRT(dot(nrm, nrm));
        real_t r = dist - (others[k].d > 0 ? others[k].d : 0);
        if (dist > (real_t)1e-6 && r > (real_t)1e-6 && r < rho0) {
            real_t mag = k_rep * (1 / r - 1 / rho0) / (r * r);
            v.x += mag * nrm.x / dist;
            v.y += mag * nrm.y / dist;
        }
    }
    if (v.x > vmax) v.x = vmax; else if (v.x < -vmax) v.x = -vmax;
    if (v.y > vmax) v.y = vmax; else if (v.y < -vmax) v.y = -vmax;
    return v;
}

void swarm_deadlock_init(swarm_deadlock_t *d, real_t hold, real_t escape, real_t angle_deg,
                         real_t v_min, real_t far_)
{
    real_t a = -angle_deg * (real_t)3.14159265358979323846 / 180;
    real_t c = COS(a), s = SIN(a);
    d->hold = hold; d->escape = escape; d->v_min = v_min; d->far = far_;
    d->R[0] = c; d->R[1] = -s; d->R[2] = s; d->R[3] = c;
    d->stall = 0; d->until = -1; d->count = 0;
}

static vec2_t rot(const swarm_deadlock_t *d, vec2_t v)
{
    return (vec2_t){ d->R[0] * v.x + d->R[1] * v.y, d->R[2] * v.x + d->R[3] * v.y };
}

vec2_t swarm_deadlock_adjust(swarm_deadlock_t *d, vec2_t v_des, vec2_t u_prev,
                             real_t dist_goal, real_t t, real_t dt)
{
    if (t < d->until) return rot(d, v_des);
    int stalled = dist_goal > d->far && SQRT(dot(u_prev, u_prev)) < d->v_min;
    d->stall = stalled ? d->stall + dt : 0;
    if (d->stall >= d->hold) {
        d->stall = 0;
        d->until = t + d->escape;
        d->count++;
        return rot(d, v_des);
    }
    return v_des;
}

real_t swarm_safe_distance(real_t base, real_t eps_self, real_t eps_nbr, real_t v_rel_max,
                           real_t age, real_t clock_err)
{
    return base + eps_self + eps_nbr + v_rel_max * age + clock_err;
}
