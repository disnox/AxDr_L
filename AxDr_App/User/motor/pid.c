#include "common.h"
/**
***********************************************************************
* @brief:      pid_para_init()
* @param[in]:  pid_config  指向pid_para_t结构体的指针
* @param[in]:  kp  比例系数
* @param[in]:  ki  积分系数
* @param[in]:  kd  微分系数
* @retval:     void
* @details:    初始化PID参数结构体
***********************************************************************
**/
void pid_para_init(pid_para_t *pid_config)
{
	pid_config->kp = 0.0f;
	pid_config->ki = 0.0f;
	pid_config->kd = 0.0f;

	pid_config->p_term = 0.0f;
	pid_config->i_term = 0.0f;
	pid_config->d_term = 0.0f;
	pid_config->ref_value = 0.0f;
	pid_config->fback_value = 0.0f;
	pid_config->error = 0.0f;
	pid_config->pre_err = 0.0f;
	pid_config->out_value = 0.0f;	
}
/**
***********************************************************************
* @brief:      pid_limit_init
* @param[in]:  pid_config  指向pid_para_t结构体的指针
* @param[in]:  i_term_max  积分项最大值
* @param[in]:  i_term_min  积分项最小值
* @param[in]:  out_max  输出最大值
* @param[in]:  out_min  输出最小值
* @retval:     void
* @details:    设置PID积分项和输出项的限制范围
***********************************************************************
**/
void pid_limit_init(pid_para_t *pid_config, float i_term_max, float i_term_min,float out_max, float out_min)
{
	pid_config->i_term_max = i_term_max;
	pid_config->i_term_min = i_term_min;
	pid_config->out_max = out_max;
	pid_config->out_min = out_min;
}
/**
***********************************************************************
* @brief:      pid_clear
* @param[in]:  pid_clear  指向pid_para_t结构体的指针
* @retval:     void
* @details:    清除PID控制器的历史状态
***********************************************************************
**/
void pid_clear(pid_para_t *pid_clear)
{
	pid_clear->ref_value = 0.0f;
	pid_clear->fback_value = 0.0f;
	pid_clear->p_term = 0.0f;
	pid_clear->i_term = 0.0f;
	pid_clear->d_term = 0.0f;
	pid_clear->error = 0.0f;
	pid_clear->pre_err = 0.0f;
	pid_clear->out_value = 0.0f;
}
/**
***********************************************************************
* @brief:      pid_reset()
* @param[in]:  pid_config  指向pid_para_t结构体的指针
* @param[in]:  kp  比例系数
* @param[in]:  ki  积分系数
* @param[in]:  kd  微分系数
* @retval:     void
* @details:    修改pid参数
***********************************************************************
**/
void pid_reset(pid_para_t *pid_config, float kp, float ki, float kd)
{
	pid_config->kp = kp;
	pid_config->ki = ki;
	pid_config->kd = kd;
}

_RAM_FUNC float parallel_pid_ctrl(pid_para_t *pid, float ref_value, float fdback_value)
{
    pid->ref_value = ref_value;
    pid->fback_value = fdback_value;

    pid->error = pid->ref_value - pid->fback_value;

    pid->p_term = pid->kp * pid->error;

    pid->d_term = pid->kd * (pid->error - pid->pre_err);
    pid->pre_err = pid->error;

    // 动态积分限幅
    pid->i_term_max = max(pid->out_max - pid->p_term, 0.0f);
    pid->i_term_min = min(pid->out_min - pid->p_term, 0.0f);

    pid->i_term += pid->ki * pid->error * pid->ts;

    if (pid->i_term > pid->i_term_max)
        pid->i_term = pid->i_term_max;
    else if (pid->i_term < pid->i_term_min)
        pid->i_term = pid->i_term_min;

    pid->out_value = pid->p_term + pid->i_term + pid->d_term;

    if (pid->out_value > pid->out_max)
        pid->out_value = pid->out_max;
    else if (pid->out_value < pid->out_min)
        pid->out_value = pid->out_min;

    return pid->out_value;
}


_RAM_FUNC float serial_pid_ctrl(pid_para_t *pid, float ref_value, float fdback_value)
{
	pid->ref_value = ref_value;
	pid->fback_value = fdback_value;
	
	pid->error = pid->ref_value - pid->fback_value;
	
	pid->p_term = pid->kp * pid->error;
	
	pid->i_term += pid->ki * pid->p_term * pid->ts;
	
	if (pid->i_term > pid->i_term_max)
		pid->i_term = pid->i_term_max;
	else if (pid->i_term < pid->i_term_min)
		pid->i_term = pid->i_term_min;
	
	pid->out_value = pid->p_term + pid->i_term;

	if (pid->out_value > pid->out_max)
		pid->out_value = pid->out_max;
	else if (pid->out_value < pid->out_min)
		pid->out_value = pid->out_min;
	
	return pid->out_value;
}

_RAM_FUNC float pdff_ctrl(pid_para_t *pid, float ref_value, float fdback_value)
{
    pid->ref_value = ref_value;
	pid->fback_value = fdback_value;
	
    float ref_temp = pid->ref_value*pid->kfp;
    float fdb_temp = pid->fback_value*(1.0f+pid->kf_damp);

    float err_kf = ref_temp - fdb_temp;

	pid->error = pid->ref_value - pid->fback_value;
	
	pid->p_term = pid->kp * err_kf;

    // 动态积分限幅
    pid->i_term_max = max(pid->out_max - pid->p_term, 0.0f);
    pid->i_term_min = min(pid->out_min - pid->p_term, 0.0f);

    pid->i_term += pid->ki * pid->error * pid->ts;

    if (pid->i_term > pid->i_term_max)
        pid->i_term = pid->i_term_max;
    else if (pid->i_term < pid->i_term_min)
        pid->i_term = pid->i_term_min;
	
	pid->d_term = pid->kd * (pid->error - pid->pre_err);
	pid->pre_err = pid->error;
	
	pid->out_value = pid->p_term + pid->i_term + pid->d_term;
	
	if (pid->out_value > pid->out_max)
		pid->out_value = pid->out_max;
	else if (pid->out_value < pid->out_min)
		pid->out_value = pid->out_min;
	
	return pid->out_value;
}

