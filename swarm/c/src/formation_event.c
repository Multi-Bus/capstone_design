/* Consensus 편대 제어, 웨이포인트 추종, 이벤트 트리거.
 * swarm/formation_control.py, swarm/event_trigger.py 의 포팅. */
#include "swarm.h"

#include <math.h>

#ifdef SWARM_DOUBLE
#define SQRT sqrt
#else
#define SQRT sqrtf
#endif

static real_t norm(vec2_t a) { return SQRT(a.x * a.x + a.y * a.y); }

vec2_t swarm_formation_vel(vec2_t p_i, vec2_t d_i, const swarm_fnbr_t *nbrs, int n,
                           vec2_t c_ref, real_t k_f, real_t k_g, real_t vmax)
{
    vec2_t u = { k_g * (c_ref.x + d_i.x - p_i.x), k_g * (c_ref.y + d_i.y - p_i.y) };
    for (int j = 0; j < n; j++) {
        u.x += k_f * (nbrs[j].p.x - p_i.x - (nbrs[j].d.x - d_i.x));
        u.y += k_f * (nbrs[j].p.y - p_i.y - (nbrs[j].d.y - d_i.y));
    }
    real_t m = norm(u);
    if (m > vmax) {
        real_t s = vmax / m;
        u.x *= s; u.y *= s;
    }
    return u;
}

void swarm_follower_init(swarm_follower_t *f, const vec2_t *wp, int n, real_t reach)
{
    f->wp = wp; f->n = n; f->i = 0; f->reach = reach;
}

static real_t dist_to(vec2_t a, vec2_t b) { return norm((vec2_t){a.x - b.x, a.y - b.y}); }

vec2_t swarm_follower_ref(swarm_follower_t *f, vec2_t center)
{
    while (f->i < f->n - 1 && dist_to(f->wp[f->i], center) < f->reach) f->i++;
    return f->wp[f->i];
}

int swarm_follower_done(const swarm_follower_t *f, vec2_t center)
{
    return f->i == f->n - 1 && dist_to(f->wp[f->n - 1], center) < f->reach;
}

/* ---- 이벤트 트리거 ---- */

void swarm_trigger_init(swarm_trigger_t *e, real_t delta, real_t max_gap)
{
    e->delta = delta; e->max_gap = max_gap; e->has_last = 0;
    e->p = (vec2_t){0, 0}; e->v = (vec2_t){0, 0}; e->t = 0;
}

vec2_t swarm_predict(vec2_t p, vec2_t v, real_t t_src, real_t t)
{
    return (vec2_t){ p.x + v.x * (t - t_src), p.y + v.y * (t - t_src) };
}

int swarm_trigger_should_send(const swarm_trigger_t *e, vec2_t p, vec2_t v, real_t t, int path_changed)
{
    (void)v;
    if (!e->has_last || path_changed || t - e->t >= e->max_gap) return 1;
    vec2_t pred = swarm_predict(e->p, e->v, e->t, t);
    return dist_to(p, pred) > e->delta;
}

void swarm_trigger_sent(swarm_trigger_t *e, vec2_t p, vec2_t v, real_t t)
{
    e->has_last = 1; e->p = p; e->v = v; e->t = t;
}

real_t swarm_effective_distance(real_t D, real_t delta)
{
    return D + 2 * delta;
}
