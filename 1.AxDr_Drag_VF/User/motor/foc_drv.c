#include "common.h"

_RAM_DATA pmsm_t pm;


/**
***********************************************************************
* @brief:      pmsm_board_init(void)
* @param[in]:  void
* @retval:     void
* @details:    驱动板参数初始化，包括电压、电流、分压电阻，放大倍数等相关参数的设置
***********************************************************************
**/
void pmsm_board_init(void)
{
    pm.board.v_ref = 3.3f;
    pm.board.v_adc = 4096.0f;

    pm.board.i_res = 0.001f;
    pm.board.i_op = 20.0f;

    pm.board.v1_res = 20000.0f; //
    pm.board.v2_res = 1000.0f;

    pm.board.v_op = (pm.board.v1_res + pm.board.v2_res) / pm.board.v2_res;
    pm.board.i_ratio = pm.board.v_ref / pm.board.v_adc / pm.board.i_res / pm.board.i_op;
    pm.board.v_ratio = pm.board.v_ref / pm.board.v_adc * pm.board.v_op;

    pm.board.i_max = pm.board.v_adc * pm.board.i_ratio * 0.5f;
    pm.board.v_max = pm.board.v_adc * pm.board.v_ratio;

    pm.board.Rt_Mos = 10000.0f;
    pm.board.Rt_Mos_res = 10000.0f;
    pm.board.Rt_Mos_Ka = 273.15f;
    pm.board.Rt_Mos_B = 3950.0f;

    pm.board.Rt_rotor = 10000.0f;
    pm.board.Rt_rotor_res = 10000.0f;
    pm.board.Rt_rotor_Ka = 273.15f;
    pm.board.Rt_rotor_B = 3950.0f;

    pm.board.dead_time = 0.5f; //us
}

/**
***********************************************************************
* @brief:      pmsm_pr60_init(void)
* @param[in]:  void
* @retval:     void
* @details:    PMSM 4310 电机参数初始化，包括极对数、电阻、电感、磁链、转动惯量等参数的设置
***********************************************************************
**/
void pmsm_2312s_init(void)
{
    pm.para.pn = 7;
    pm.para.Rs = 0.202977806f;
    pm.para.Ld = 0.000108778855f;
    pm.para.Lq = 0.000112416135f;
    pm.para.Ls = 0.000110597495f;
    pm.para.Ldif = 3.63728032e-06f;
    pm.para.flux = 0.006488f;
    pm.para.B  = 0.000188353f;
    pm.para.Js = 9.08865259e-05f;

    pm.para.Gr = 1.0f;
    pm.para.ibw = 1000.0f;
    pm.para.delta = 4.0f;

    pm.para.div_pn = 1.0f / pm.para.pn;
    pm.para.pnd_2pi = pm.para.pn / M_2PI;
    pm.para.div_Gr = 1.0f / pm.para.Gr;
    pm.para.Gref = 1.0f;

    pm.para.Kt = 1.5f * pm.para.pn * pm.para.flux;
    pm.para.div_Kt = 1.0f / pm.para.Kt;

    pm.ctrl.wm_acc = 200.0f;
    pm.ctrl.wm_dec = 200.0f;

    pm.para.e_off = 2.34354496f;
    pm.para.r_off = 2.12998796f;
    pm.para.m_off = 0.0f;
}

/**
***********************************************************************
* @brief:      pmsm_peroid_init(void)
* @param[in]:  void
* @retval:     void
* @details:    电机周期参数初始化，包括FOC、PID等相关周期和采样时间的设置
***********************************************************************
**/
void pmsm_peroid_init(void)
{
    pm.period.foc_fs = 20000.0f;
    pm.period.foc_ts = 0.00005f;

    pm.period.cur_pid_fs = 20000.0f;
    pm.period.cur_pid_ts = 1.0f/pm.period.cur_pid_fs;
    pm.period.cur_pid_cnt_val = pm.period.foc_fs * pm.period.cur_pid_ts;

    pm.period.spd_pid_fs = 10000.0f;
    pm.period.spd_pid_ts = 1.0f/pm.period.spd_pid_fs;
    pm.period.spd_pid_cnt_val = pm.period.foc_fs * pm.period.spd_pid_ts;

    pm.period.pos_pid_fs = 5000.0f;
    pm.period.pos_pid_ts = 1.0f/pm.period.pos_pid_fs;
    pm.period.pos_pid_cnt_val = pm.period.foc_fs * pm.period.pos_pid_ts;

    pm.period.spd_mea_fs = 1000.0f;
    pm.period.spd_mea_ts = 0.001f;
    pm.period.spd_mea_cnt_val = pm.period.foc_fs * pm.period.spd_mea_ts;
}

/**
***********************************************************************
* @brief:      pmsm_init(void)
* @param[in]:  void
* @retval:     void
* @details:    PMSM参数及控制器初始化，包括电机、板级、保护、周期、滤波器等参数的设置及相关初始化函数的调用
***********************************************************************
**/
void pmsm_init(void)
{
    memset(&pm, 0, sizeof(pm));
    pmsm_2312s_init();

    pmsm_board_init();
    pmsm_peroid_init();

    pm.ctrl_bit = start;

    foc_get_curr_off();
}

/**
***********************************************************************
* @brief:      foc_spd_measure_M(float pos, float fs)
* @param[in]:  pos 当前位置（角度/弧度）
* @param[in]:  fs  采样频率
* @retval:     float 速度值
* @details:    速度测量函数，根据当前位置和采样频率计算速度
***********************************************************************
**/
_RAM_FUNC float foc_spd_measure_M(float pos, float fs)
{
    static float pos_dif;
    static float pos_last;

    pos_dif = pos - pos_last;
    wrap_pm_pi(pos_dif);

    float vel = pos_dif * fs;

    pos_last = pos;

    return vel;
}
/**
***********************************************************************
* @brief:      foc_para_calc(pmsm_t* pm)
* @param[in]:  pm 指向 PMSM 参数结构体的指针
* @retval:     void
* @details:    FOC相关参数计算，包括母线电压、母线电流、滤波电流、转矩等参数的计算
***********************************************************************
**/
_RAM_FUNC void foc_para_calc(pmsm_t* pm)
{
    pm->foc.vbus = ((float)pm->adc.vbus * pm->board.v_ratio);
    pm->foc.inv_vbus = 1.5f / (pm->foc.vbus);
    pm->foc.vs = pm->foc.vbus*0.5f*0.96f;

    pm->foc.we = foc_spd_measure_M(pm->foc.p_e, 20000);
    pm->foc.wr = pm->foc.we * pm->para.div_pn; // rad/s;
}

/**
***********************************************************************
* @brief:      get_curr_off(void)
* @param[in]:  void
* @retval:     void
* @details:    电流零偏采集，采集三相ADC的偏置值并求平均，存入pm.adc结构体
***********************************************************************
**/
void foc_get_curr_off(void)
{
    float sum_a, sum_b, sum_c;
    for (int i = 0; i < 1000; i++)
    {
        HAL_Delay(1);
        sum_a += (float)(ADC1->JDR3);
        sum_b += (float)(ADC1->JDR2);
        sum_c += (float)(ADC1->JDR1);
    }

    pm.adc.ia_off = sum_a * 0.001f;
    pm.adc.ib_off = sum_b * 0.001f;
    pm.adc.ic_off = sum_c * 0.001f;
}



/**
***********************************************************************
* @brief:      foc_pwm_start(void)
* @param[in]:  void
* @retval:     void
* @details:    启动三路PWM输出，分别对应三相电机控制
***********************************************************************
**/
_RAM_FUNC void foc_pwm_start(void)
{
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);
}

/**
***********************************************************************
* @brief:      foc_pwm_stop(void)
* @param[in]:  void
* @retval:     void
* @details:    停止三路PWM输出，关闭三相电机控制
***********************************************************************
**/
_RAM_FUNC void foc_pwm_stop(void)
{
    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
    HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);
    HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3);
    HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_3);
}

extern uint16_t adc1_buff[2];
extern uint16_t adc2_buff[4];
_RAM_FUNC void foc_adc_sample(pmsm_t* pm)
{
	pm->adc.ia = ADC1->JDR3;
	pm->adc.ib = ADC1->JDR2;
	pm->adc.ic = ADC1->JDR1;

	pm->adc.va = adc2_buff[1];
	pm->adc.vb = adc2_buff[2];
	pm->adc.vc = adc2_buff[3];
    pm->adc.vbus     = ADC2->JDR1;

    //  Convert ADC values to actual currents with offset compensation and scaling
    pm->foc.i_a = ((float) pm->adc.ia - pm->adc.ia_off) * pm->board.i_ratio;
    pm->foc.i_b = ((float) pm->adc.ib - pm->adc.ib_off) * pm->board.i_ratio;
    pm->foc.i_c = ((float) pm->adc.ic - pm->adc.ic_off) * pm->board.i_ratio;
}

/**
***********************************************************************
* @brief:      foc_pwm_run(foc_para_t* foc)
* @param[in]:  foc 指向 FOC 参数结构体的指针
* @retval:     void
* @details:    根据duty周期设置三相PWM输出，实现SVPWM调制
***********************************************************************
**/
_RAM_FUNC void foc_pwm_run(pmsm_t* pm)
{
	htim1.Instance->CCR1 = (uint16_t)(pm->foc.dtc_a * PWM_ARR());
	htim1.Instance->CCR2 = (uint16_t)(pm->foc.dtc_b * PWM_ARR());
	htim1.Instance->CCR3 = (uint16_t)(pm->foc.dtc_c * PWM_ARR());
}

_RAM_FUNC void foc_pwm_duty_set(pmsm_t* pm)
{
    htim1.Instance->CCR1 = (uint16_t)(0.5f * PWM_ARR());
    htim1.Instance->CCR2 = (uint16_t)(0.5f * PWM_ARR());
    htim1.Instance->CCR3 = (uint16_t)(0.5f * PWM_ARR());
}


/**
***********************************************************************
* @brief:      foc_volt(pmsm_t* pm, float vd_ref, float vq_ref, float pos)
* @param[in]:  pm 指向 PMSM 参数结构体的指针
* @param[in]:  vd_ref d轴电压参考值
* @param[in]:  vq_ref q轴电压参考值
* @param[in]:  pos    电机电气位置（角度/弧度）
* @retval:     void
* @details:    电压控制，设置d/q轴电压参考值，完成Clarke、Park变换及SVPWM输出
***********************************************************************
**/
_RAM_FUNC void foc_volt(pmsm_t* pm, float vd_ref, float vq_ref, float pos)
{
	pm->foc.mode = foc_volt_mode;
    clarke_transform(&pm->foc);
    pm->foc.theta = pos;
    wrap_0_2pi(pm->foc.theta);
    sin_cos_val(&pm->foc);
    park_transform(&pm->foc);
    pm->foc.v_d = vd_ref;
    pm->foc.v_q = vq_ref;
    inverse_park(&pm->foc);
    if(svm(pm->foc.v_alph * (pm->foc.inv_vbus), pm->foc.v_beta * (pm->foc.inv_vbus), &pm->foc.dtc_a, &pm->foc.dtc_b, &pm->foc.dtc_c)==0) {
        foc_pwm_run(pm);
    }
}


