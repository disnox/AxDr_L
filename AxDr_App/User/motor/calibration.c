#include "common.h"

void cali_init(void)
{
    pm.calibr.vd_set = 0.8f;
    pm.calibr.state = cali_pp_start;

}

_RAM_FUNC void cali_mag_encoder(pmsm_t *pm)
{
    cali_t *x = &pm->calibr;

    x->lut_num = 256;
    x->window  = 128;

    x->raw = pm->pos_box.raw_1;
    x->pos = pm->pos_box.pos_1;
    x->bit = pm->pos_box.bit_1;
    foc_volt(pm, x->vd_ref, x->vq_ref, x->pe_set);
    
    switch (x->state)
    {
        case cali_pp_start:
            memset(x->lut, 0, sizeof(x->lut));
            if (++x->i < 20000) // align rotor
            {
                x->vd_ref = x->vd_set;
                x->pe_set = 0.0f;
                x->pe_acc = 0.002f;
                x->pos_start = x->pos;
                x->dir = 1;
            }
            else
            {
                x->pe_set += x->pe_acc;

                if (x->pe_set > 0.5f * M_PI)
                {
                    if (x->pe_set < 0.55f * M_PI)
                        x->pos_dir_end = x->pos;

                    if (fabsf(x->pos_start - x->pos) < 0.001f) // 比较位置
                        x->state = cali_pp_calc;
                }
            }
            break;
        case cali_pp_calc:
        {
            x->pn = x->pe_set / M_2PI;
            pm->para.pn = x->pn;

            float pos_dif = x->pos_dir_end - x->pos_start;
            wrap_pm_pi(pos_dif);

            if (pos_dif < 0) {
                // 编码器反馈值递减，需要调换V和W两相
                if(pm->para.phase_order == ABC_PHASE) {
                    pm->para.phase_order = ACB_PHASE;
                    x->order = ACB_PHASE;
                } else {
                    pm->para.phase_order = ABC_PHASE;
                    x->order = ABC_PHASE;
                }
                x->dir = -1;
            } else {
                x->dir = 1;
            }

            x->state = cali_lut_init;
            break;
        }
        case cali_lut_init:
            if (++x->i < 20000) // align rotor
                x->pe_set = 0.0f;
            else
            {
                x->n1 = x->lut_num * x->pn;
                x->n2 = 40;
                x->pe_acc = M_2PI * x->pn / (x->n1 * x->n2);

                x->i = 0; x->j = x->n2;
                x->state = cali_lut_cw;
            }
            break;
        case cali_lut_cw:
        {
            if (x->i < (x->n1 * x->n2))
            {
                if (x->j == x->n2)
                {
                    x->j = 0;
                    x->pos_err = x->pr_set - x->pos; // rad
                    wrap_pm_pi(x->pos_err);
                    if (x->k == 0) x->p_raw[0] = x->raw;
                    x->p_error_arr[x->k] = x->pos_err;
                    x-> k++;
                }
                x->pe_set += x->pe_acc; //[0,2π]*pn
                x->pr_set  = x->pe_set / x->pn; // [0,2π]*pn
                wrap_0_2pi(x->pr_set);
                x->i++; x->j++;
            }
            else
            {
                x->i = 0;
                x->j = x->n2; //k=0;
                x->state = cali_lut_ccw;
            }
            break;
        }
        case cali_lut_ccw:
        {
            if (x->i < (x->n1 * x->n2))
            {
                if (x->j == x->n2)
                {
                    x->j = 0;

                    x->pos_err = x->pr_set - x->pos; // rad
                    wrap_pm_pi(x->pos_err);

                    if (x->k == 2 * x->n1 - 1)
                        x->p_raw[1] = x->raw;
                    x->p_error_arr[x->k] = x->pos_err;
                    x->k++;
                }
                x->pe_set -= x->pe_acc; // [0,2π]*pn
                x->pr_set  = x->pe_set / x->pn; // [0,2π]*pn
                wrap_0_2pi(x->pr_set);
                x->i++; x->j++;
            }
            else
            {
                x->i = 0; x->j = 0;
                x->vd_ref = 0.0f;
                x->state = cali_off_calc;
            }
            break;
        }
        case cali_off_calc:
        {
            for (int i = 0; i < x->n1; i++)
            {
                //Average the forward and back directions
                x->p_error_arr[i] = 0.5f * (x->p_error_arr[i] + x->p_error_arr[2 * x->n1 - i - 1]);
                x->offset = x->offset + x->p_error_arr[i] / x->n1;
            }
            x->e_off = fmodf(x->offset * x->pn, M_2PI);
            x->r_off = fmodf(x->offset, M_2PI);
            wrap_pm_pi(x->e_off);
            x->state = cali_mean_calc;
            break;
        }
        case cali_mean_calc:
        {
            float inv_window = 1.0f / (float)x->window;
            int half_window = x->window * 0.5f;

            for (int i = 0; i < x->n1; i++)
            {
                float temp = 0.0f;
                for (int j = 0; j < x->window; j++)
                {
                    x->ind = -half_window + j + i; // Indexes from -window/2 to + window/2
                    if (x->ind < 0)
                        x->ind += x->n1;
                    else if (x->ind > (x->n1 - 1))
                        x->ind -= x->n1;
                    temp += x->p_error_arr[x->ind]*inv_window;
                }
                if (i % x->pn == 0)
                    x->p_err[i / x->pn] = temp; // 转存到error数组中

                x->mean += temp / x->n1;
            }
            x->state = cali_lut_calc;
            break;
        }
        case cali_lut_calc:
        {
            // TODO_4: Rebuild lut using error_filt
            x->raw_off = (x->p_raw[0] + x->p_raw[1]) * 0.5f;
            //Insensitive to errors in this direction, so 2 points is plenty
            if (x->i < x->lut_num)
            {
                // build lookup table
                x->ind = (x->raw_off >> (x->bit-8)) + x->i;
                if (x->ind > (x->lut_num - 1))
                    x->ind -= x->lut_num;
                x->lut[x->ind] = (x->p_err[x->i] - x->mean) * ((x->bit>>1)/M_2PI);
                x->i++;
            }
            else
            {
                if (++x->j < x->lut_num)
                {
                    pm->map.enc_lut = x->lut[x->j];
                    pm->map.enc_table[x->j] = x->lut[x->j];
                    x->j++;
                }
                else
                    x->state = cali_lut_end;
            }
            break;
        }
        case cali_lut_end:
            x->state = cali_pp_start;
            pm->ctrl_bit = reset;
            pm->mode.sys = release_mode;
            //pm->pos_box.dir_1 = x->dir;
            pm->para.pn = x->pn;
            //pm->para.phase_order = x->order;
            pm->para.e_off = x->e_off;
            pm->para.r_off = x->r_off;
            pm->flag.bit.cali_sensor1_done = 1;

            // ---- reset variables ----
//            cali_reset_state(x);
            break;
    }
}

_RAM_FUNC void cali_reset_state(cali_t *x)
{
    x->dir   = 0;
    x->pn    = 0;
    x->raw   = 0;
    x->bit   = 0;
    x->pos   = 0.0f;

    x->vd_ref = 0.0f;
    x->vq_ref = 0.0f;

    x->pos_err   = 0.0f;
    x->pe_set    = 0.0f;
    x->pr_set    = 0.0f;
    x->pe_acc    = 0.0f;

    x->pos_start   = 0.0f;
    x->pos_end     = 0.0f;
    x->pos_dir_end = 0.0f;

    // lut_num 和 window 保留
    memset(x->lut, 0, sizeof(x->lut));

    x->raw_off = 0;
    x->offset  = 0.0f;

    x->n1 = 0;
    x->n2 = 0;

    x->e_off = 0.0f;
    x->r_off = 0.0f;

    x->i = 0;
    x->j = 0;
    x->k = 0;

    memset(x->p_raw, 0, sizeof(x->p_raw));
    memset(x->p_err, 0, sizeof(x->p_err));
    memset(x->p_error_arr, 0, sizeof(x->p_error_arr));

    x->mean = 0.0f;
    x->ind  = 0;
}






