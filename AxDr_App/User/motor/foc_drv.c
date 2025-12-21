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
* @brief:      pmsm_protect_init(void)
* @param[in]:  void
* @retval:     void
* @details:    电机保护参数初始化，包括过流、过压、欠压、过温等保护阈值的设置
***********************************************************************
**/
void pmsm_protect_init(void)
{
    pm.protect.oc_value = 80.0f; // A
    pm.protect.ov_value = 60.0f; // V
    pm.protect.uv_value = 15.0f; // V
    pm.protect.ot_value = 100.0f; //
    pm.protect.omt_value = 100.0f; //
    pm.protect.time_value = 0; // 50us

    pm.protect.oc_time = 0.1f; // s
    pm.protect.ov_time = 0.1f; // s
    pm.protect.uv_time = 0.1f; // s
    pm.protect.ot_time = 0.1f; // s
    pm.protect.omt_time = 0.1f; // s
    pm.protect.link_out_time = 0.1f; // s

    pm.protect.oc_cnt_value = pm.protect.oc_time * pm.period.foc_fs;
    pm.protect.ov_cnt_value = pm.protect.ov_time * pm.period.foc_fs;
    pm.protect.uv_cnt_value = pm.protect.uv_time * pm.period.foc_fs;
    pm.protect.ot_cnt_value = pm.protect.ot_time * pm.period.foc_fs;
    pm.protect.omt_cnt_value = pm.protect.omt_time * pm.period.foc_fs;
    pm.protect.link_out_cnt_value = pm.protect.link_out_time * pm.period.foc_fs;
}

/**
***********************************************************************
* @brief:      pmsm_pr60_init(void)
* @param[in]:  void
* @retval:     void
* @details:    PMSM 4310 电机参数初始化，包括极对数、电阻、电感、磁链、转动惯量等参数的设置
***********************************************************************
**/
void pmsm_pr60_init(void)
{
    pm.para.rated_voltage = 24.0f;
    pm.para.rated_current = 0.0f;
    pm.para.rated_speed   = 3000.0f/9.55f;
    pm.para.rated_torque  = 0.8f;
    pm.para.rated_power   = 0.0f;
    pm.para.peak_current  = 0.0f;
    pm.para.peak_torque   = 2.0f;
    pm.para.peak_speed    = 3000.0f/9.55f;

    pm.para.pn = 10;
    pm.para.Rs = 0.162977806f;
    pm.para.Ld = 0.000108778855f;
    pm.para.Lq = 0.000112416135f;
    pm.para.Ls = 0.000110597495f;
    pm.para.Ldif = 3.63728032e-06f;
    pm.para.flux = 0.00498822471f;
    pm.para.B  = 0.000188353f;
    pm.para.Js = 7.32527915e-05f;

    pm.para.Gr = 1.0f;
    pm.para.ibw = 500.0f;
    pm.para.delta = 4.0f;

    pm.para.div_pn = 1.0f / pm.para.pn;
    pm.para.pnd_2pi = pm.para.pn / M_2PI;
    pm.para.div_Gr = 1.0f / pm.para.Gr;
    pm.para.Gref = 1.0f;

    pm.para.Kt = 1.5f * pm.para.pn * pm.para.flux;
    pm.para.div_Kt = 1.0f / pm.para.Kt;

    pm.ctrl.wm_acc = 20.0f;
    pm.ctrl.wm_dec = 20.0f;

    pm.para.phase_order = ABC_PHASE;
    pm.para.e_off = 1.33748674f;
    pm.para.r_off = -0.494569868f;
    pm.para.m_off = 0.0f;

    pm.app_ctrl.pmax_torm =  pm.para.peak_torque*0.8f;
    pm.app_ctrl.nmax_torm = -1.0f*pm.para.peak_torque*0.8f;
    pm.app_ctrl.pmax_velm =  pm.para.peak_speed*0.8f;
    pm.app_ctrl.nmax_velm = -1.0f*pm.para.peak_speed*0.8f;
    pm.app_ctrl.pmax_posm =  20000.0f;
    pm.app_ctrl.nmax_posm = -1.0f*20000.0f;

    pm.ctrl.pmax_tor =  pm.app_ctrl.pmax_torm*pm.para.div_Gr;
    pm.ctrl.nmax_tor =  pm.app_ctrl.nmax_torm*pm.para.div_Gr;
    pm.ctrl.pmax_iq  =  pm.ctrl.pmax_tor*pm.para.div_Kt;
    pm.ctrl.nmax_iq  =  pm.ctrl.nmax_tor*pm.para.div_Kt;

    pm.ctrl.pmax_vel =  pm.app_ctrl.pmax_velm*pm.para.Gr;
    pm.ctrl.nmax_vel =  pm.app_ctrl.nmax_velm*pm.para.Gr;
    pm.ctrl.pmax_pos =  pm.app_ctrl.pmax_posm*pm.para.Gr;
    pm.ctrl.nmax_pos =  pm.app_ctrl.nmax_posm*pm.para.Gr;
}

void pmsm_2312s_init(void)
{
	pm.para.rated_voltage = 24.0f;
    pm.para.rated_current = 0.0f;
    pm.para.rated_speed   = 10000.0f/9.55f;
    pm.para.rated_torque  = 0.8f;
    pm.para.rated_power   = 0.0f;
    pm.para.peak_current  = 0.0f;
    pm.para.peak_torque   = 2.0f;
    pm.para.peak_speed    = 10000.0f/9.55f;
	
    pm.para.pn = 7;
    pm.para.Rs = 0.108945489f;
    pm.para.Ld = 1.97248246e-05f;
    pm.para.Lq = 2.02818483e-05f;
    pm.para.Ls = 2.00033355e-07f;
    pm.para.Ldif = 5.57023668e-07f;
    pm.para.flux = 0.000884152076f;
    pm.para.B  = 0.000188353f;
    pm.para.Js = 2.19904655e-06f;

    pm.para.Gr = 1.0f;
    pm.para.ibw = 500.0f;
    pm.para.delta = 4.0f;

    pm.para.div_pn = 1.0f / pm.para.pn;
    pm.para.pnd_2pi = pm.para.pn / M_2PI;
    pm.para.div_Gr = 1.0f / pm.para.Gr;
    pm.para.Gref = 1.0f;

    pm.para.Kt = 1.5f * pm.para.pn * pm.para.flux;
    pm.para.div_Kt = 1.0f / pm.para.Kt;

    pm.ctrl.wm_acc = 200.0f;
    pm.ctrl.wm_dec = 200.0f;

    pm.para.phase_order = ACB_PHASE;
    pm.para.e_off = 2.34354496f;
    pm.para.r_off = 2.12998796f;
    pm.para.m_off = 0.0f;
	
	pm.app_ctrl.pmax_torm =  pm.para.peak_torque*0.8f;
    pm.app_ctrl.nmax_torm = -1.0f*pm.para.peak_torque*0.8f;
    pm.app_ctrl.pmax_velm =  pm.para.peak_speed*0.8f;
    pm.app_ctrl.nmax_velm = -1.0f*pm.para.peak_speed*0.8f;
    pm.app_ctrl.pmax_posm =  20000.0f;
    pm.app_ctrl.nmax_posm = -1.0f*20000.0f;

    pm.ctrl.pmax_tor =  pm.app_ctrl.pmax_torm*pm.para.div_Gr;
    pm.ctrl.nmax_tor =  pm.app_ctrl.nmax_torm*pm.para.div_Gr;
    pm.ctrl.pmax_iq  =  pm.ctrl.pmax_tor*pm.para.div_Kt;
    pm.ctrl.nmax_iq  =  pm.ctrl.nmax_tor*pm.para.div_Kt;

    pm.ctrl.pmax_vel =  pm.app_ctrl.pmax_velm*pm.para.Gr;
    pm.ctrl.nmax_vel =  pm.app_ctrl.nmax_velm*pm.para.Gr;
    pm.ctrl.pmax_pos =  pm.app_ctrl.pmax_posm*pm.para.Gr;
    pm.ctrl.nmax_pos =  pm.app_ctrl.nmax_posm*pm.para.Gr;
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
* @brief:      pmsm_lpf_init(void)
* @param[in]:  void
* @retval:     void
* @details:    低通滤波器参数初始化，包括各通道滤波器的截止频率和采样频率设置，并调用初始化函数
***********************************************************************
**/
void pmsm_lpf_init(void)
{
    pm.id_lpf.fc = 200.0f; // Hz
    pm.id_lpf.fs = 20000.0f;
    pm.iq_lpf.fc = 200.0f; // Hz
    pm.iq_lpf.fs = 20000.0f;
    pm.vd_lpf.fc = 200.0f; // Hz
    pm.vd_lpf.fs = 20000.0f;
    pm.vq_lpf.fc = 200.0f; // Hz
    pm.vq_lpf.fs = 20000.0f;

    pm.ibus_lpf.fc = 200.0f; // Hz
    pm.ibus_lpf.fs = 20000.0f; // Hz
    pm.vbus_lpf.fc = 200.0f;
    pm.vbus_lpf.fs = 20000.0f;

    pm.iabs_lpf.fc = 200.0f; // Hz
    pm.iabs_lpf.fs = 20000.0f; // Hz

    pm.wr_lpf.fc = 200.0f; // Hz
    pm.wr_lpf.fs = 20000.0f; // Hz

    low_pf_init(&pm.id_lpf);
    low_pf_init(&pm.iq_lpf);
    low_pf_init(&pm.vd_lpf);
    low_pf_init(&pm.vq_lpf);
    low_pf_init(&pm.ibus_lpf);
    low_pf_init(&pm.iabs_lpf);
    low_pf_init(&pm.vbus_lpf);
    low_pf_init(&pm.wr_lpf);
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
    pmsm_lpf_init();

    pmsm_protect_init();

    foc_cur_pi_calc(&pm);
    foc_spd_pi_calc(&pm);
	
//	pm.iq_pi.kp = 0;
//	pm.iq_pi.ki = 0;
//	pm.spd_pi.kp = 0;
//	pm.spd_pi.ki = 0;

    pm.pos_pi.kp = 12.0f;//pm.spd_pi.kp * 1.4f * pm.period.pos_pid_fs;//24.0f;

    pid_limit_init(&pm.id_pi, 11.0f, -11.0f, 11.0f, -11.0f);
    pid_limit_init(&pm.iq_pi, 11.0f, -11.0f, 11.0f, -11.0f);
    pid_limit_init(&pm.spd_pi, 20.0f, -20.0f, 20.0f, -20.0f);
    pid_limit_init(&pm.pos_pi, 200.0f, -200.0f, 200.0f, -200.0f);
    
    iden_init();
    scvm_init();
    nlob_init();
    alob_init();
    encoder_init();
    cali_init();
    traj_init();

    eh_observer_init();

    pm.ctrl_bit = start;

    pm.pos_box.pos_mode = Sensorsory_s; //Sensorsory_s; Sensorsory_d; //Sensorless;
    pm.pos_box.sensory1 = MT6816; //DMENC; //MT6825; //MT6816; //MA732; // DMENC; //Hall; //Xhall;
    pm.pos_box.senless  = Scvm;       //Nlob; //Alob; //Scvm; //Esmo; //Hfsi;

    pm.mode.sys      = debug_mode;        //debug_mode; //release_mode; // calibrat_mode;
    pm.mode.debug    = curr_cl;   //drag_vf; //volt_op; //drag_if; //curr_cl; //spd_curr_cl; //pos_spd_curr_cl;
    pm.mode.release  = vel_mode;        // mit_mode; //tor_mode; //vel_mode; //pos_mode;
    pm.mode.calibrat = iden_pm; // rotor_enc_cali; //iden_pm; //enc_mod;

    pm.app_ctrl.p_curve = tcurve;
    pm.app_ctrl.v_curve = tcurve;
    pm.app_ctrl.vel_set_immediate = 1;
    pm.app_ctrl.pos_set_immediate = 1;

    foc_get_curr_off();
}


/**
***********************************************************************
* @brief:      foc_cur_pi_calc(pmsm_t* pm)
* @param[in]:  pm 指向 PMSM 参数结构体的指针
* @retval:     void
* @details:    电流环 PI 参数计算，包括 kp、ki、ts 的设置
***********************************************************************
**/
_RAM_FUNC void foc_cur_pi_calc(pmsm_t* pm)
{
    pm->id_pi.kp = pm->para.Ls * pm->para.ibw;
    pm->id_pi.ki = pm->para.Rs * pm->para.ibw;
    pm->id_pi.ts = pm->period.cur_pid_ts;

    pm->iq_pi.kp = pm->para.Ls * pm->para.ibw;
    pm->iq_pi.ki = pm->para.Rs * pm->para.ibw;
    pm->iq_pi.ts = pm->period.cur_pid_ts;
}

/**
***********************************************************************
* @brief:      foc_spd_pi_calc(pmsm_t* pm)
* @param[in]:  pm 指向 PMSM 参数结构体的指针
* @retval:     void
* @details:    速度环 PI 参数计算，包括 kp、ki、ts 的设置
***********************************************************************
**/
_RAM_FUNC void foc_spd_pi_calc(pmsm_t* pm)
{
    pm->spd_pi.kfp = 1.1f;
    pm->spd_pi.kf_damp = 0.25f;

    float K = (3.0f * pm->para.pn * pm->para.flux) / (4.0f * pm->para.Js);

    pm->spd_pi.kp = (pm->iq_pi.kp/pm->para.Ls)/(pm->para.delta*K);
    pm->spd_pi.ki = ((pm->iq_pi.kp/pm->para.Ls)*(pm->iq_pi.kp/pm->para.Ls))/(pm->para.delta*pm->para.delta*pm->para.delta*K);
    pm->spd_pi.ts = pm->period.spd_pid_ts;
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


/**
***********************************************************************
* @brief:      foc_curr(pmsm_t* pm, float id_set, float iq_set, float pos)
* @param[in]:  pm      指向 PMSM 参数结构体的指针
* @param[in]:  id_set  d轴电流设定值
* @param[in]:  iq_set  q轴电流设定值
* @param[in]:  pos     电机电气位置（角度/弧度）
* @retval:     void
* @details:    电流环控制，完成Clarke、Park变换，PI调节，SVPWM输出等
***********************************************************************
**/
_RAM_FUNC void foc_curr(pmsm_t* pm, float id_set, float iq_set, float pos)
{
	pm->foc.mode = foc_curr_mode;
    clarke_transform(&pm->foc);
    pm->foc.theta = pos;
    wrap_0_2pi(pm->foc.theta);
    sin_cos_val(&pm->foc);

    park_transform(&pm->foc);

    if (++pm->period.cur_pid_cnt >= pm->period.cur_pid_cnt_val)
    {
        parallel_pid_ctrl(&pm->id_pi, id_set, pm->foc.i_d);
        pm->foc.v_d = pm->id_pi.out_value;
        parallel_pid_ctrl(&pm->iq_pi, iq_set, pm->foc.i_q);
        pm->foc.v_q = pm->iq_pi.out_value;
    }

    inverse_park(&pm->foc);
    if(0 == svm(pm->foc.v_alph * (pm->foc.inv_vbus), pm->foc.v_beta * (pm->foc.inv_vbus), &pm->foc.dtc_a, &pm->foc.dtc_b, &pm->foc.dtc_c)) {
        foc_pwm_run(pm);
    }
}
/**
***********************************************************************
* @brief:      foc_vel(pmsm_t* pm, float vel_set, float iq_lim, float pos)
* @param[in]:  pm      指向 PMSM 参数结构体的指针
* @param[in]:  vel_set 速度设定值
* @param[in]:  iq_set  q轴电流设定值，实则是电流限制值
* @param[in]:  pos     电机电气位置（角度/弧度）
* @retval:     void
* @details:    速度环控制，完成Clarke、Park变换，PI调节，电流限制，SVPWM输出等
***********************************************************************
**/
_RAM_FUNC void foc_vel(pmsm_t* pm, float vel_set, float iq_set, float pos)
{
	pm->foc.mode = foc_vel_mode;
    clarke_transform(&pm->foc);
    pm->foc.theta = pos;
    wrap_0_2pi(pm->foc.theta);
    sin_cos_val(&pm->foc);
    park_transform(&pm->foc);

    if (++pm->period.spd_pid_cnt >= pm->period.spd_pid_cnt_val)
    {
        pm->period.spd_pid_cnt = 0;
        pdff_ctrl(&pm->spd_pi, vel_set, pm->foc.wr_f);
        pm->ctrl.iq_lim = pm->spd_pi.out_value;

        if (ABS(iq_set) > 0)
            pm->ctrl.iq_lim = sat1_datf(pm->ctrl.iq_lim, ABS(iq_set), -ABS(iq_set));
    }

    if (++pm->period.cur_pid_cnt >= pm->period.cur_pid_cnt_val)
    {
        parallel_pid_ctrl(&pm->id_pi, pm->ctrl.id_set, pm->foc.i_d);
        pm->foc.v_d = pm->id_pi.out_value;
        parallel_pid_ctrl(&pm->iq_pi, pm->ctrl.iq_lim, pm->foc.i_q);
        pm->foc.v_q = pm->iq_pi.out_value;
    }

    inverse_park(&pm->foc);
    if(0 == svm(pm->foc.v_alph * (pm->foc.inv_vbus), pm->foc.v_beta * (pm->foc.inv_vbus), &pm->foc.dtc_a, &pm->foc.dtc_b, &pm->foc.dtc_c)) {
        foc_pwm_run(pm);
    }
}

/**
***********************************************************************
* @brief:      foc_pos(pmsm_t* pm, float pos_set, float vel_lim, float iq_lim, float pos)
* @param[in]:  pm       指向 PMSM 参数结构体的指针
* @param[in]:  pos_set  位置设定值
* @param[in]:  vel_set  速度设定值，实则是速度限制
* @param[in]:  iq_set   q轴电流设定值，实则是电流限制值
* @param[in]:  pos      电机电气位置（角度/弧度）
* @retval:     void
* @details:    位置环控制，完成Clarke、Park变换，PI调节，速度/电流限制，SVPWM输出等
***********************************************************************
**/
_RAM_FUNC void foc_pos(pmsm_t* pm, float pos_set, float vel_set, float iq_set, float pos)
{
	pm->foc.mode = foc_pos_mode;
    clarke_transform(&pm->foc);
    pm->foc.theta = pos;
    wrap_0_2pi(pm->foc.theta);
    sin_cos_val(&pm->foc);
    park_transform(&pm->foc);

    if (++pm->period.pos_pid_cnt >= pm->period.pos_pid_cnt_val)
    {
        pm->period.pos_pid_cnt = 0;
        parallel_pid_ctrl(&pm->pos_pi, pos_set, pm->foc.mp_r);
        pm->ctrl.wr_lim = pm->pos_pi.out_value;

        if (ABS(vel_set) > 0)
            pm->ctrl.wr_lim = sat1_datf(pm->ctrl.wr_lim, ABS(vel_set), -ABS(vel_set));
    }

    if (++pm->period.spd_pid_cnt >= pm->period.spd_pid_cnt_val)
    {
        pm->period.spd_pid_cnt = 0;
        pdff_ctrl(&pm->spd_pi, pm->ctrl.wr_lim, pm->foc.wr_f);
        pm->ctrl.iq_lim = pm->spd_pi.out_value;

        if (ABS(iq_set) > 0)
            pm->ctrl.iq_lim = sat1_datf(pm->ctrl.iq_lim, ABS(iq_set), -ABS(iq_set));
    }

    if (++pm->period.cur_pid_cnt >= pm->period.cur_pid_cnt_val)
    {
        parallel_pid_ctrl(&pm->id_pi, pm->ctrl.id_set, pm->foc.i_d);
        pm->foc.v_d = pm->id_pi.out_value;
        parallel_pid_ctrl(&pm->iq_pi, pm->ctrl.iq_lim, pm->foc.i_q);
        pm->foc.v_q = pm->iq_pi.out_value;
    }

    inverse_park(&pm->foc);
    if (0 == svm(pm->foc.v_alph * (pm->foc.inv_vbus), pm->foc.v_beta * (pm->foc.inv_vbus), &pm->foc.dtc_a, &pm->foc.dtc_b, &pm->foc.dtc_c)) 
    {
         foc_pwm_run(pm);
    }
}

/**
***********************************************************************
* @brief:      spd_measure_M(float pos, float fs)
* @param[in]:  pos 当前位置（角度/弧度）
* @param[in]:  fs  采样频率
* @retval:     float 速度值
* @details:    速度测量函数，根据当前位置和采样频率计算速度
***********************************************************************
**/
_RAM_FUNC float spd_measure_M(float pos, float fs)
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

    pm->id_pi.out_max    =  pm->foc.vs;
    pm->id_pi.out_min    = -pm->foc.vs;
    pm->iq_pi.out_max    =  pm->foc.vs;
    pm->iq_pi.out_min    = -pm->foc.vs;

    pm->spd_pi.out_max   =  pm->ctrl.pmax_iq;
    pm->spd_pi.out_min   =  pm->ctrl.nmax_iq;

    pm->pos_pi.out_max   =  pm->ctrl.pmax_vel;
    pm->pos_pi.out_min   =  pm->ctrl.nmax_vel;

    //	pm->foc.vbus_f = low_pf(&pm->vbus_lpf, pm->foc.vbus);
    //	pm->foc.id_f = low_pf(&pm->id_lpf, pm->foc.i_d);
    pm->foc.iq_f   = low_pf(&pm->iq_lpf, pm->foc.i_q);
    pm->foc.tor_r  = pm->foc.i_q    * pm->para.Kt;
    pm->foc.tor_rf = pm->foc.iq_f   * pm->para.Kt;
    pm->foc.tor_m  = pm->foc.tor_r  * pm->para.Gr;
    pm->foc.tor_mf = pm->foc.tor_rf * pm->para.Gr;
    //	pm->foc.vd_f = low_pf(&pm->vd_lpf, pm->foc.v_d);
    //	pm->foc.vq_f = low_pf(&pm->vq_lpf, pm->foc.v_q);

    //	pm->foc.ibus   = (pm->foc.v_d * pm->foc.i_d + pm->foc.v_q * pm->foc.i_q)*pm->foc.inv_vbus;
    //	pm->foc.ibus_f = low_pf(&pm->ibus_lpf, pm->foc.ibus);
    //
    //	pm->foc.i_abs  = sqrtf(SQ(pm->foc.i_d) + SQ(pm->foc.i_q));
    //	pm->foc.iabs_f = low_pf(&pm->iabs_lpf, pm->foc.i_abs);

    //	pm->foc.duty_now = SIGN(pm->foc.v_q) * NORM2_f(pm->foc.v_d, pm->foc.v_q) * TWO_BY_SQRT3*pm->foc.inv_vbus;

    pm->foc.we = spd_measure_M(pm->foc.p_e, 20000);
    pm->foc.wr = pm->foc.we * pm->para.div_pn; // rad/s;

    pm->foc.wr_f = low_pf(&pm->wr_lpf, pm->foc.wr);
    pm->foc.wm = pm->foc.wr_f * pm->para.div_Gr; // rad/s
                                                                                                                  
    eh_speed_observer(&eh_vobs, pm->foc.wr, pm->foc.tor_r);
    eh_torque_observer(&eh_tobs, pm->foc.we, pm->foc.tor_r);
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
	
    
	
	switch (pm.para.phase_order) {
    case ABC_PHASE:
        pm.adc.ia_off = sum_a * 0.001f;
		pm.adc.ib_off = sum_b * 0.001f;
		pm.adc.ic_off = sum_c * 0.001f;
        break;
    case ACB_PHASE:
        pm.adc.ia_off = sum_a * 0.001f;
		pm.adc.ib_off = sum_c * 0.001f;
		pm.adc.ic_off = sum_b * 0.001f;
        break;
    default:
        // 默认相序
        break;
    }
}

/**
***********************************************************************
* @brief:      foc_clear(void)
* @param[in]:  void
* @retval:     void
* @details:    FOC相关参数清零，包括控制参数和FOC结构体的重置
***********************************************************************
**/
_RAM_FUNC void foc_clear(pmsm_t* pm)
{
    pm->ctrl.vd_set       = 0.0f;
    pm->ctrl.vq_set       = 0.0f;
    pm->ctrl.id_set       = 0.0f;
    pm->ctrl.iq_set       = 0.0f;
    pm->ctrl.iq_lim       = 0.0f;
    pm->ctrl.torm_set     = 0.0f;
    pm->ctrl.tor_set      = 0.0f;
    pm->ctrl.we_set       = 0.0f;
    pm->ctrl.wr_set       = 0.0f;
    pm->ctrl.wr_lim       = 0.0f;
    pm->ctrl.wm_set       = 0.0f;
    pm->ctrl.wm_ref       = 0.0f;
    pm->ctrl.wm_lim       = 0.0f;
    pm->ctrl.wm_diff      = 0.0f;
    pm->ctrl.posm_set     = 0.0f;
    pm->ctrl.posm_ref     = 0.0f;
    pm->ctrl.posr_set     = 0.0f;
    pm->ctrl.wm_lst       = 0.0f;
    pm->ctrl.posm_lst     = 0.0f;
    pm->ctrl.mit_tor_set  = 0.0f;
    pm->ctrl.kp           = 0.0f;
    pm->ctrl.kd           = 0.0f;
}

// 计算温度的函数
void temp_calc(void)
{
    float mos_ntc_volt  = (pm.adc.Tmos_bc / (float)pm.board.v_adc) * pm.board.v_ref;  // 转换ADC读数为电压
    
    pm.board.Rt_Mos = mos_ntc_volt*pm.board.Rt_Mos_res/(3.3f-mos_ntc_volt);
    pm.foc.Tmos = 1.0f/(1.0f/(pm.board.Rt_Mos_Ka+25.0f) + logf(pm.board.Rt_Mos/pm.board.Rt_Mos_res)/pm.board.Rt_Mos_B) - pm.board.Rt_Mos_Ka + 0.5;

    float rotor_ntc_volt  = (pm.adc.Trotor / (float)pm.board.v_adc) * pm.board.v_ref;  // 转换ADC读数为电压
    
    pm.board.Rt_rotor = rotor_ntc_volt*pm.board.Rt_rotor_res/(3.3f-rotor_ntc_volt);
    pm.foc.Tcoil = 1.0f/(1.0f/(pm.board.Rt_rotor_Ka+25.0f) + logf(pm.board.Rt_rotor/pm.board.Rt_rotor_res)/pm.board.Rt_rotor_B) - pm.board.Rt_rotor_Ka + 0.5;
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
    switch (pm->para.phase_order) {
    case ABC_PHASE:
        pm->adc.ia = ADC1->JDR3;
        pm->adc.ib = ADC1->JDR2;
        pm->adc.ic = ADC1->JDR1;

        pm->adc.va = adc2_buff[1];
        pm->adc.vb = adc2_buff[2];
        pm->adc.vc = adc2_buff[3];
        break;
    case ACB_PHASE:
        pm->adc.ia = ADC1->JDR3;
        pm->adc.ic = ADC1->JDR2;
        pm->adc.ib = ADC1->JDR1;

        pm->adc.va = adc2_buff[1];
        pm->adc.vc = adc2_buff[2];
        pm->adc.vb = adc2_buff[3];
        break;
    default:
        break;
    }
    pm->adc.vbus     = ADC2->JDR1;
    // pm->adc.Trotor   = adc3_seq_buff[4] & 0x0000FFFF;
    // pm->adc.Tmos_ab  = adc3_seq_buff[5] & 0x0000FFFF;
    // pm->adc.Tmos_bc  = adc3_seq_buff[6] & 0x0000FFFF;
    // pm->adc.sin_hall = adc3_seq_buff[7] & 0x0000FFFF;
    // pm->adc.cos_hall = adc3_seq_buff[8] & 0x0000FFFF;

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
    switch (pm->para.phase_order) {
    case ABC_PHASE:
        htim1.Instance->CCR1 = (uint16_t)(pm->foc.dtc_a * PWM_ARR());
        htim1.Instance->CCR2 = (uint16_t)(pm->foc.dtc_b * PWM_ARR());
        htim1.Instance->CCR3 = (uint16_t)(pm->foc.dtc_c * PWM_ARR());
        break;
    case ACB_PHASE:
        htim1.Instance->CCR1 = (uint16_t)(pm->foc.dtc_a * PWM_ARR());
        htim1.Instance->CCR2 = (uint16_t)(pm->foc.dtc_c * PWM_ARR());
        htim1.Instance->CCR3 = (uint16_t)(pm->foc.dtc_b * PWM_ARR());
        break;
    default:
        // 默认相序
        break;
    }
}

_RAM_FUNC void foc_pwm_duty_set(pmsm_t* pm)
{
    htim1.Instance->CCR1 = (uint16_t)(0.5f * PWM_ARR());
    htim1.Instance->CCR2 = (uint16_t)(0.5f * PWM_ARR());
    htim1.Instance->CCR3 = (uint16_t)(0.5f * PWM_ARR());
}
