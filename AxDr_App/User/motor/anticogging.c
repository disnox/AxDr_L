//
// Created by disno on 2025/9/18.
//

#include "main.h"

void anticog_init(void)
{
    pm.anticog.map_num = 3840;

    pm.anticog.wr_set = 10.0f;
    pm.anticog.step_value = 800;
    pm.anticog.gap_number = 100;

    pm.anticog.delta_p = M_2PI / (float)pm.anticog.map_num;
}

_RAM_FUNC void anticogging_calibration(pmsm_t *pm)
{
    anticog_t *x = &pm->anticog;
    x->wr_set = 10.0f; // 转速，建议不要太大
    pm->ctrl.posr_set = x->posr_set;
    pm->ctrl.wr_set   = x->wr_set;
    pm->ctrl.iq_set   = 0.0f;

    foc_pos(pm, pm->ctrl.posr_set, pm->ctrl.wr_set, pm->ctrl.iq_set, pm->foc.p_e);

    if (++x->count < 700) {
        return;
    }
    x->count = 0;
    x->number++;

    if (x->number <= x->gap_number)
    {
        // 阶段1：预旋转（CW）
        x->posr_set += x->delta_p;
    }
    else if (x->number <= (x->gap_number + x->map_num))
    {
        // 阶段2：CW 扫描
        float pos_mod = fmodf(x->posr_set, M_2PI);
        if (pos_mod < 0.0f) pos_mod += M_2PI;

        uint16_t index = (uint16_t)(pos_mod * div_M_2PI * (float)x->map_num + 0.5f);
        if (index >= x->map_num) index = 0;

        pm->map.aco_table[index] = pm->foc.i_q; // 记录正向电流
        pm->map.aco_lut = pm->foc.i_q;

        x->posr_set += x->delta_p;
    }
    else
    {
        x->wr_set = 0.0f;  // 停止电机
        pm->flag.bit.anticog_done = 1; // 置完成标志
        pm->ctrl_bit = reset;
        pm->mode.sys = release_mode;
    }
}


//     // 计算 index
//     float index_f = pm->foc.sp_r * div_M_2PI * (float)COGGING_MAP_NUM;
//     uint16_t index0 = (uint16_t)index_f;
//     // 将补偿电流加入目标
//     pm->ctrl.iq_lim += map[index0];

