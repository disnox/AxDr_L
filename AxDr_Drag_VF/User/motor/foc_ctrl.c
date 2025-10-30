#include "common.h"
#include "modlue.h"
/**
***********************************************************************
* @brief:      pwmv2_cmp1_callback(void)
* @param[in]:  void
* @retval:     void
* @details:    PWM主中断回调函数，读取ADC值，计算电流，执行FOC参数计算、故障检测和状态控制
***********************************************************************
**/
_RAM_FUNC void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    foc_adc_sample(&pm);
    foc_para_calc(&pm);
    pmsm_state_ctrl(&pm);
	vofa_start();
}

/**
***********************************************************************
* @brief:      pmsm_state_ctrl(pmsm_t* pm)
* @param[in]:  pm  指向永磁同步电机（PMSM）控制结构体的指针
* @retval:     void
* @details:    PMSM 状态机控制，根据当前状态执行启动、预充、复位和运行等操作
***********************************************************************
**/
_RAM_FUNC void pmsm_state_ctrl(pmsm_t* pm)
{
    // State machine for PMSM control
    switch (pm->ctrl_bit)
    {
    case start:
        // Initialize PWM and transition to precharge state
        foc_pwm_start();
        foc_pwm_duty_set(pm);
        break;
    case reset:
         //Reset all controllers and stop PWM
        foc_pwm_stop();
        break;
    case opera:
        // Normal operation mode control
        pmsm_mode_ctrl(pm);
        break;
    default:
        break;
    }
}

/**
***********************************************************************
* @brief:      pmsm_mode_ctrl(pmsm_t* pm)
* @param[in]:  pm  指向永磁同步电机（PMSM）控制结构体的指针
* @retval:     void
* @details:    PMSM模式控制，根据当前系统模式选择不同的控制策略
***********************************************************************
**/
_RAM_FUNC void pmsm_mode_ctrl(pmsm_t* pm)
{
	force_volt_mode(pm); // V/f control mode
}

/**
***********************************************************************
* @brief:      force_volt_mode(pmsm_t* pm)
* @param[in]:  pm  指向永磁同步电机（PMSM）控制结构体的指针
* @retval:     void
* @details:    V/f 控制模式下的电压控制，计算电角速度和位置，执行电压控制
***********************************************************************
**/
_RAM_FUNC void force_volt_mode(pmsm_t* pm)
{
    // Calculate electrical angular velocity from reference speed
    pm->ctrl.we_set = pm->ctrl.wr_set * pm->para.pn;
    // Calculate position increment per FOC period
    pm->ctrl.pos_acc = pm->ctrl.we_set * pm->period.foc_ts;
    // Update electrical angle
    pm->ctrl.drag_pe += pm->ctrl.pos_acc;
    // Wrap angle to [0, 2π)
    wrap_0_2pi(pm->ctrl.drag_pe);
    // Apply voltage control
    foc_volt(pm, pm->ctrl.vd_set, pm->ctrl.vq_set, pm->ctrl.drag_pe);
}


