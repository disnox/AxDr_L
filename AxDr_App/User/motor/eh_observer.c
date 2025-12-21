//
// Created by disno on 2025/7/30.
//
#include "common.h"

__attribute__((section(".fast_ram"))) eh_vobs_t eh_vobs;
__attribute__((section(".fast_ram"))) eh_tobs_t eh_tobs;
_RAM_FUNC void eh_observer_init(void)
{
    eh_vobs.wn = 20.0f;
    eh_vobs.g1 = 2.0f*eh_vobs.wn;
    eh_vobs.g2 = 3.0f*eh_vobs.wn*eh_vobs.wn;
    eh_vobs.g3 = -pm.para.Js*eh_vobs.wn*eh_vobs.wn*eh_vobs.wn;
    eh_vobs.kj = 100.0f;
    eh_vobs.tf = 1.0f;
    eh_vobs.wr_obs = 0;
    eh_vobs.pr_obs = 0;
    eh_vobs.ts = 0.00005f;

    eh_tobs.g1 = 1000.0f;
    eh_tobs.g2 = -250.0f;
    eh_tobs.a[0][0] = 0;
    eh_tobs.a[0][1] = -1/pm.para.Js;
    eh_tobs.a[1][0] = 0;
    eh_tobs.a[1][1] = 0;
    eh_tobs.b[0] = 1/pm.para.Js;
    eh_tobs.b[1] = 0;
    eh_tobs.c[0] = 1;
    eh_tobs.c[1] = 0;

    eh_tobs.ts = 0.00005f;
}

_RAM_FUNC void eh_speed_observer(eh_vobs_t *x, float wr, float tor)
{
    float x2, x3, x4, u1;

    x->wkbf2 = u1*(x->g1*x->tf-1);
    x->wkbf1 = x->wr_obs + x->wkbf2;

    u1  = wr - x->wkbf1;
    x2  = u1 * x->tf * x->g2;
    x3 += u1 * x->tf * x->g3 * x->ts;
    x4  = tor* x->kj;

    x->acc = x->wr_obs+x2+x3+x4;
    x->wr_obs += x->acc*x->ts;
}


_RAM_FUNC void eh_torque_observer(eh_tobs_t *x, float we, float tor)
{
    x->u = tor;
    x->x[0] = x->we_obs;
    x->x[1] = x->Tl_obs;

// 计算 Z = A*x + B*u + g*(we - C*x)
// 计算 A*x
    x->z[0] += x->a[0][0] * x->x[0];
    x->z[0] += x->a[0][1] * x->x[1];
    x->z[1] += x->a[1][0] * x->x[0];
    x->z[1] += x->a[1][1] * x->x[1];

// 计算 B*u
    x->z[0] += x->b[0] * x->u;
    x->z[1] += x->b[1] * x->u;

// 计算 C*x
    float cx = 0;
    cx += x->c[0] * x->x[0];
    cx += x->c[1] * x->x[1];

// 计算 g*(we - C*x)
    x->z[0] += x->g1 * (we - cx);
    x->z[1] += x->g2 * (we - cx);

    x->we_obs = x->z[0]*x->ts;
    x->Tl_obs = x->z[1]*x->ts;
}



