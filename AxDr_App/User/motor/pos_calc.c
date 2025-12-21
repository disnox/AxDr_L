//
// Created by disno on 2025/8/19.
//
#include "common.h"

_RAM_FUNC void send_encoder_read_command(pmsm_t* pm)
{
    switch(pm->pos_box.pos_mode) {
        case Sensorsory_s:
            switch (pm->pos_box.sensory1) {
                case MA732:  read_ma732_raw(); break;
                case MT6816: read_mt6816_raw(); break;
                case MT6825: read_mt6825_raw(); break;
                case DMENC:  read_dm485enc_raw(); break; // 如果是DM485编码器
                default:     break;
            }
            break;
        case Sensorsory_d:
            switch (pm->pos_box.sensory1) {
                case MA732:  read_ma732_raw(); break;
                case MT6816: read_mt6816_raw(); break;
                case MT6825: read_mt6825_raw(); break;
                case DMENC:  read_dm485enc_raw(); break; // 如果是DM485编码器
                default:     break;
            }
            switch (pm->pos_box.sensory2) {
                case MA732:  read_ma732_raw(); break;   // 如有第二路SPI/IO
                case MT6816: read_mt6816_raw(); break;
                case MT6825: read_mt6825_raw(); break;
                default:     break;
            }
            break;
    }
}

_RAM_FUNC void pos_calc(pmsm_t* pm)
{
    enc_para_t* x1 = NULL;
    enc_para_t* x2 = NULL;

    // 根据编码器类型读取原始值
    switch (pm->pos_box.pos_mode)
    {
        case Sensorsory_s: // 单编码器有感
            switch (pm->pos_box.sensory1) {
                case MT6825: x1 = &pm->pos_box.mt6825;   break;
                case MT6816: x1 = &pm->pos_box.mt6816;   break;
                case MA732:  x1 = &pm->pos_box.ma732;    break;
                case DMENC:  x1 = &pm->pos_box.dm485enc; break;
                default: break;
            }

            if (x1->rev_flag) {
                x1->rev_flag = 0;
                pos_encoder_calc(x1, pm);
                pm->foc.e_pr = x1->pos; // 主编码器结果
                sensory1_pos_calc(pm);

                pm->pos_box.raw_1 = x1->raw;
                pm->pos_box.bit_1 = x1->bit;
                pm->pos_box.pos_1 = x1->pos;
            }
            break;

        case Sensorsory_d: // 副编码器
            switch (pm->pos_box.sensory1) {
                case MT6825: x1 = &pm->pos_box.mt6825;   break;
                case MT6816: x1 = &pm->pos_box.mt6816;   break;
                case MA732:  x1 = &pm->pos_box.ma732;    break;
                case DMENC:  x1 = &pm->pos_box.dm485enc; break;
                default: break;
            }
            switch (pm->pos_box.sensory2) {
                case MT6825: x2 = &pm->pos_box.mt6825; break; // 如有独立enc_para_t建议新建
                case MT6816: x2 = &pm->pos_box.mt6816; break;
                case MA732:  x2 = &pm->pos_box.ma732;  break;
                default: break;
            }

            if (x1->rev_flag && x2->rev_flag) {
                x1->rev_flag = 0;
                x2->rev_flag = 0;
                pos_encoder_calc(x2, pm);
                pm->foc.e_pr = x1->pos; // 转子侧用主编码器
                pm->foc.e_pm = x2->pos; // 机械输出轴侧角度用副编码器
                sensory2_pos_calc(pm);

                pm->pos_box.raw_1 = x1->raw;
                pm->pos_box.bit_1 = x1->bit;
                pm->pos_box.pos_1 = x1->pos;
                pm->pos_box.raw_2 = x2->raw;
                pm->pos_box.bit_2 = x2->bit;
                pm->pos_box.pos_2 = x2->pos;
            }
            break;

        case Sensorless:
            switch (pm->pos_box.senless)
            {
                case Nlob:
                    nlob_vesc(&pm->nlob);
                    pm->foc.p_e = pm->nlob.pos_e;
                    pm->ctrl.id_set = pm->nlob.id_out;
                    break;
                case Alob:
                    alob_flux(&pm->alob);
                    pm->foc.p_e = pm->alob.pos_e;
                    pm->ctrl.id_set = pm->alob.id_out;
                    break;
                case Scvm:
                    scvm_obe(&pm->scvm);
                    pm->foc.p_e = pm->scvm.pos_e;
                    pm->ctrl.id_set = pm->scvm.id_out;
                    break;
                case Esmo:
                    break;
                case Hsfi:
                    break;
                case Ekf:
                    break;
                default:
                    return; // 不支持的编码器类型
            }
            senless_pos_calc(pm);
            break;
        default: return; // 不支持的角度类型
    }
}

/**
***********************************************************************
* @brief:      sensory_pos_calc(pmsm_t* pm)
* @param[in]:  pm  指向永磁同步电机（PMSM）控制结构体的指针
* @retval:     void
* @details:    计算有传感器模式下的位置和角度，包括转数、电气角度、机械角度等
***********************************************************************
**/

_RAM_FUNC void sensory1_pos_calc(pmsm_t* pm)
{
    pmsm_foc_t* x = &pm->foc;
    pmsm_para_t* y = &pm->para;

    // 补偿读取角度过程中的滞后
    //if(ABS(x->wr_f) > 50.0f)
//    x->e_pr += (x->wr_f * 40.0f *  0.000001f);
        
    // 计算位置差异和转数
    x->pr_dif = x->e_pr - x->pr_lst;
    if (x->pr_dif >  0.88f * M_2PI) x->rev--;
    if (x->pr_dif < -0.88f * M_2PI) x->rev++;

    x->e_pe = x->e_pr * y->pn - (uint32_t)(x->e_pr * y->pnd_2pi) * M_2PI + y->e_off;
    wrap_0_2pi(x->e_pe);

    x->p_e  = x->e_pr * y->pn - (uint32_t)(x->e_pr * y->pnd_2pi) * M_2PI + y->e_off;
    wrap_0_2pi(x->p_e);

    x->sp_r = x->e_pr + y->r_off;
    wrap_0_2pi(x->sp_r);
    x->mp_r = x->e_pr + x->rev * M_2PI + y->r_off;

    x->sp_m = x->sp_r * y->div_Gr + y->m_off;
    wrap_0_2pi(x->sp_m);
    x->mp_m = x->mp_r * y->div_Gr;

    x->m_rev = (int32_t)(x->mp_m * div_M_2PI);

    x->pr_lst = x->e_pr;
}

_RAM_FUNC void sensory2_pos_calc(pmsm_t* pm)
{
    pmsm_foc_t* x = &pm->foc;
    pmsm_para_t* y = &pm->para;

    // 计算位置差异和转数
    x->pr_dif = x->e_pr - x->pr_lst;

    if (x->pr_dif >  0.8f * M_2PI) x->rev--;
    if (x->pr_dif < -0.8f * M_2PI) x->rev++;

    // 计算电气角度
    x->e_pe = x->e_pr * y->pn - (uint32_t)(x->e_pr * y->pnd_2pi) * M_2PI;
    wrap_0_2pi(x->e_pe);
    x->p_e  = x->e_pr * y->pn - (uint32_t)(x->e_pr * y->pnd_2pi) * M_2PI + y->e_off;
    wrap_0_2pi(x->p_e);

    x->sp_r = x->e_pr + y->r_off;
    wrap_0_2pi(x->sp_r);
    x->mp_r = x->e_pr + x->rev * M_2PI + y->r_off;

    // 计算减速后单圈、多圈机械角度
    // 计算位置差异和转数
    x->pm_dif = x->e_pm - x->pm_lst;

    if (x->pm_dif >  0.8f * M_2PI) x->m_rev--;
    if (x->pm_dif < -0.8f * M_2PI) x->m_rev++;

    x->sp_m = x->e_pm + y->m_off;
    wrap_0_2pi(x->sp_m);
    x->mp_m = x->sp_m + x->m_rev * M_2PI;

    x->pr_lst = x->e_pr;
    x->pm_lst = x->e_pm;
}

/**
***********************************************************************
* @brief:      senless_pos_calc(pmsm_t* pm)
* @param[in]:  pm  指向永磁同步电机（PMSM）控制结构体的指针
* @retval:     void
* @details:    无传感器模式下的位置和角度计算，通过电角度反推转子角度，计算多圈机械角度和圈数
***********************************************************************
**/
_RAM_FUNC void senless_pos_calc(pmsm_t* pm)
{
    pmsm_foc_t* x = &pm->foc;
    pmsm_para_t* y = &pm->para;

    // 由电角度反推转子角度
    x->sp_r = x->p_e * y->div_pn;
    wrap_0_2pi(x->sp_r);

    // 计算位置差异和转数
    x->pr_dif = x->sp_r - x->pr_lst;
    if (x->pr_dif >  0.8f * M_2PI) x->rev--;
    if (x->pr_dif < -0.8f * M_2PI) x->rev++;

    wrap_0_2pi(x->sp_r);

    // 多圈转子角度
    x->mp_r = x->sp_r + x->rev * M_2PI;

    // 计算减速后单圈、多圈机械角度
    x->sp_m = x->sp_r * y->div_Gr;
    wrap_0_2pi(x->sp_m);
    x->mp_m = x->mp_r * y->div_Gr;

    // 计算减速后机械圈数
    x->m_rev = (int32_t)(x->mp_m / M_2PI);

    x->pr_lst = x->sp_r;
}

