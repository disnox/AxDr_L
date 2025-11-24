#ifndef __COMMON_H__
#define __COMMON_H__

#include <float.h>
#include <math.h>
#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "main.h"

#define _RAM_FUNC   __attribute__((section(".RamFunc")))
#define _RAM_DATA   __attribute__((section(".data")))

<<<<<<< HEAD
//ATTR_RAMFUNC
//ATTR_PLACE_AT_FAST_RAM
//ATTR_RAMFUNC

extern uint32_t run_tick, sys_freq;
extern float run_us, tickinlus;
=======
>>>>>>> 157aa8c814c3698051e702968095d8070515b611

// Type definitions for various data types
typedef float f32;
typedef double f64;
typedef int64_t s64;
typedef int32_t s32;
typedef int16_t s16;
typedef int8_t s8;
typedef uint64_t u64;
typedef uint32_t u32;
typedef uint16_t u16;
typedef uint8_t u8;

typedef const float cf32;
typedef const double cf64;
typedef const int64_t cs64;
typedef const int32_t cs32;
typedef const int16_t cs16;
typedef const int8_t cs8;
typedef const uint64_t cu64;
typedef const uint32_t cu32;
typedef const uint16_t cu16;
typedef const uint8_t cu8;

typedef __IO float vf32;
typedef __IO double vf64;
typedef __IO int64_t vs64;
typedef __IO int32_t vs32;
typedef __IO int16_t vs16;
typedef __IO int8_t vs8;
typedef __IO uint64_t vu64;
typedef __IO uint32_t vu32;
typedef __IO uint16_t vu16;
typedef __IO uint8_t vu8;

typedef __I float vcf32;
typedef __I double vcf64;
typedef __I int64_t vcs64;
typedef __I int32_t vcs32;
typedef __I int16_t vcs16;
typedef __I int8_t vcs8;
typedef __I uint64_t vcu64;
typedef __I uint32_t vcu32;
typedef __I uint16_t vcu16;
typedef __I uint8_t vcu8;

// Macros for mathematical operations
#define SIGN(x) (((x) < 0.0f) ? -1.0f : 1.0f) // Return the sign of the argument
#define NORM2_f(x, y) (sqrtf(SQ(x) + SQ(y)))  // Two-norm of 2D vector

// Check for NaN and infinity in floats
#define UTILS_IS_INF(x) ((x) == (1.0f / 0.0f) || (x) == (-1.0f / 0.0f))
#define UTILS_IS_NAN(x) ((x) != (x))
#define UTILS_NAN_ZERO(x) (x = UTILS_IS_NAN(x) ? 0.0f : x)

#define MIN_MAX_LIMT(in, low, high)                                            \
  (in = in > high ? high : in < low ? low : in)
#define MAX_LIMT(in, outmax)                                                   \
  (in = in > outmax ? outmax : in < (-outmax) ? (-outmax) : in)
#define SQ(x) ((x) * (x))
#define ABS(x) ((x) > 0 ? (x) : -(x))
// #define MAX(x, y)     (((x) > (y)) ? (x) : (y))
// #define MIN(x, y)     (((x) < (y)) ? (x) : (y))
#define min(x, y) (((x) < (y)) ? (x) : (y)) // Get minimum value
#define max(x, y) (((x) > (y)) ? (x) : (y)) // Get maximum value
#define CLAMP(x, lower, upper) (MIN(upper, MAX(x, lower)))
#define FLOAT_EQU(floatA, floatB) ((ABS((floatA) - (floatB))) < 0.000001f)

#define wrap_pm_pi(theta)                                                      \
  theta = (theta > M_PI) ? theta - M_2PI : theta;                              \
  theta = (theta < -M_PI) ? theta + M_2PI : theta;
#define wrap_0_2pi(theta)                                                      \
  theta = (theta > M_2PI) ? theta - M_2PI : theta;                             \
  theta = (theta < 0.0f) ? theta + M_2PI : theta;

// Mathematical constants
#define M_PI (3.14159265358f)         // Pi
#define M_2PI (6.28318530716f)        // 2 * Pi
<<<<<<< HEAD
=======
#define M_2_PI (6.28318530716f)        // 2 * Pi
>>>>>>> 157aa8c814c3698051e702968095d8070515b611
#define div_M_2PI (0.159154943092391467f)        // 1/(2 * Pi)
#define SQRT3 (1.73205080757f)        // Square root of 3
#define SQRT3_BY_2 (0.86602540378f)   // Square root of 3 divided by 2
#define ONE_BY_SQRT3 (0.57735026919f) // 1 divided by square root of 3
#define TWO_BY_SQRT3 (1.15470053838f) // 2 divided by square root of 3

// Bit manipulation macros
#define setbit(x, y) x |= (1 << y)
#define clrbit(x, y) x &= ~(1 << y)
#define reversebit(x, y) x ^= (1 << y)
#define getbit(x, y) ((x) >> (y) & 1)

<<<<<<< HEAD
#define PWM_ARR() __HAL_TIM_GET_AUTORELOAD(&htim1)

#define cs_down       HAL_GPIO_WritePin(SPI1_CSN_GPIO_Port, SPI1_CSN_Pin, GPIO_PIN_RESET);
#define cs_up         HAL_GPIO_WritePin(SPI1_CSN_GPIO_Port, SPI1_CSN_Pin, GPIO_PIN_SET);

=======
#define Dead_Time 80
#define PWM_ARR() __HAL_TIM_GET_AUTORELOAD(&htim1)

#define set_dtc_a(value) __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, value)
#define set_dtc_b(value) __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, value)
#define set_dtc_c(value) __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, value)

#define cs_down       HAL_GPIO_WritePin(SPI1_CSN_GPIO_Port, SPI1_CSN_Pin, GPIO_PIN_RESET);
#define cs_up         HAL_GPIO_WritePin(SPI1_CSN_GPIO_Port, SPI1_CSN_Pin, GPIO_PIN_SET);


>>>>>>> 157aa8c814c3698051e702968095d8070515b611
typedef enum
{
	foc_volt_mode,
	foc_curr_mode,
	foc_vel_mode,
	foc_pos_mode,
<<<<<<< HEAD
} foc_mode_e;

// 相序枚举
typedef enum {
    ABC_PHASE = 0,      // 默认相序 A-B-C
    ACB_PHASE = 1,      // A-C-B
    BAC_PHASE = 2,      // B-A-C
    BCA_PHASE = 3,      // B-C-A
    CAB_PHASE = 4,      // C-A-B
    CBA_PHASE = 5,      // C-B-A
} phase_order_e;

// Low-pass filter structure
typedef struct
{
    float val;
    float val_f;
    float fs;
    float fc;
    float filt_a;
    float filt_b;
} lpf_t;

// PID parameter structure
typedef struct
{
    volatile float kp; // Proportional gain
    volatile float ki; // Integral gain
    volatile float kd; // Derivative gain

    volatile float kfp; // Proportional gain
    volatile float kf_damp; // Integral gain

    volatile float p_term; // Proportional term
    volatile float i_term; // Integral term
    volatile float d_term; // Derivative term

    volatile float i_term_max; // Maximum integral term
    volatile float i_term_min; // Minimum integral term

    volatile float ts; // Sampling time

    volatile float ref_value; // Reference value
    volatile float fback_value; // Feedback value

    volatile float error; // Error
    volatile float pre_err; // Previous error

    volatile float out_min; // Minimum output
    volatile float out_max; // Maximum output

    volatile float out_value;
} pid_para_t;

// PLL parameter structure
typedef struct
{
    float kp;
    float ki;
    float wn;
    float damp;
    float ts;

    float err;
    float last_err;
    float sin_val;
    float cos_val;

    float p_term; // Proportional term
    float i_term; // Integral term
    float i_term_max;
    float out_max;
    float out_value;

    float pos_e;
    float we;
} pll_t;

=======

} foc_mode_e;

>>>>>>> 157aa8c814c3698051e702968095d8070515b611
// FOC parameter structure
typedef struct
{
	foc_mode_e mode;
    float vs; // Voltage space vector
    float vbus;
    float ibus;
    float vbus_f;
    float ibus_f;
    float inv_vbus;

    float i_abs;
    float iabs_f;
    float duty_now;

    int rev; // Rotor revolution
    int m_rev; // Mechanical revolution
    float e_pr; // Encoder Position
    float e_pm;
    float e_pe;

    float p_e; // Electrical position
    float sp_r; // Rotor position
    float mp_r; // Rotor position
    float mp_m; // Mechanical position
    float sp_m; // Position
    float we; // Electrical velocity
    float wr; // Rotor velocity
    float wr_f;
    float wm; // Mechanical velocity
    float spd_mea; // Speed measurement
    float tor_r;
    float tor_rf;
    float tor_m;
    float tor_mf;
    float Tcoil; // Coil temperature
    float Tmos; // MOS temperature

    float pr_dif; // Position difference
    float pr_lst; // Last position
    float pm_dif; // Position difference
    float pm_lst; // Last position

    float theta;
    float sin_val;
    float cos_val;

    float i_a;
    float i_b;
    float i_c;

    float v_a;
    float v_b;
    float v_c;

    float va_f;
    float vb_f;
    float vc_f;

    float i_d;
    float i_q;

    float id_f;
    float iq_f;

    float v_d;
    float v_q;

    float vd_f;
    float vq_f;

    float i_alph;
    float i_beta;

    float ialph_f;
    float ibeta_f;

    float v_alph;
    float v_beta;

    float valph_f;
    float vbeta_f;

    float dtc_a;
    float dtc_b;
    float dtc_c;
} pmsm_foc_t;

// Power management state enumeration
typedef enum
{
    reset = 0,
    start = 1,
    opera = 2,
} pm_ctrl_bit_e;

typedef enum
{
    stop = 0,
    prech = 1,
    runing = 2,
<<<<<<< HEAD
    fault = 3
} pm_state_bit_e;

typedef enum
{
    debug_mode    = 0,
    release_mode  = 1,
    calibrat_mode = 2,
    fault_mode    = 3,
} sys_mode_e;

typedef enum
{
    drag_vf = 0,
    drag_if = 1,
    volt_op = 2,
    curr_cl = 3,
    spd_volt_cl = 4,
    spd_curr_cl = 5,
    pos_spd_curr_cl = 6,
    pos_spd_volt_cl = 7,
} debug_mode_e;

typedef enum
{
    mit_mode = 0,
    tor_mode = 1,
    vel_mode = 2,
    pos_mode = 3,
} release_mode_e;

typedef enum
{
    normal_mode = 0,
    quick_mode = 1,
} fault_mode_e;

typedef enum
{
    enc_mod  = 0,
    enc_cali = 1,
    iden_pm  = 2
} calibrat_mode_e;

// 新增：mode控制结构体，统一管理所有mode
typedef struct
{
    sys_mode_e       sys;
    debug_mode_e     debug;
    release_mode_e   release;
    calibrat_mode_e  calibrat;
    fault_mode_e     fault;
} mode_ctrl_t;

typedef enum
{
    Sensorsory_s = 0,   // 单
    Sensorsory_d = 1,
    Sensorless   = 2,
}  pos_mode_e;

typedef enum
{
    MA732  = 1,
    MT6816 = 2,
    MT6825 = 3,
    Hall   = 4,
    Xhall  = 5,
    DMENC  = 6,
} encoder_type_e;

typedef enum
{
    Nlob = 1,
    Alob = 2,
    Scvm = 3,
    Esmo = 4,
    Hsfi = 5,
    Ekf  = 6,
} senless_type_e;

typedef struct
{
    pos_mode_e     pos_mode;   // 位置模式：有感/无感
    senless_type_e senless;    // 无感类型
    encoder_type_e sensory1;    // 主编码器类型
    encoder_type_e sensory2;   // 第二编码器类型（如有）
} pos_type_t;

typedef enum
{
    tcurve = 0, // 梯形控制曲线
    scurve = 1  // S形控制曲线
} curve_type_e;

typedef enum
{
    abs_pos_mode = 0, // 绝对位置模式
    rel_pos_mode = 1  // 相对位置模式
} pos_ctrl_mode_e;

=======
    fault = 3,
} pm_state_bit_e;

>>>>>>> 157aa8c814c3698051e702968095d8070515b611
// PMSM parameter structure
typedef struct
{
    float rated_voltage;   // 额定电压
<<<<<<< HEAD
    float rated_current;   // 额定电流
    float rated_speed;     // 额定转速
    float rated_torque;    // 额定扭矩
    float rated_power;     // 额定功率
    float peak_current;    // 峰值电流
    float peak_torque;     // 峰值扭矩
    float peak_speed;      // 峰值转速

    phase_order_e phase_order;
=======
    float rated_current;   // 额定母线电流
    float rated_speed;     // 额定转速
    float rated_torque;    // 额定扭矩
    float rated_power;     // 额定功率
    float peak_current;    // 峰值母线电流
    float peak_torque;     // 峰值扭矩
    float peak_speed;      // 峰值转速
	
>>>>>>> 157aa8c814c3698051e702968095d8070515b611
    float Rs; // Resistance
    float Ls; // Inductance
    float Ld; // d-axis inductance
    float Lq; // q-axis inductance
    float Ldif; // dq-axis inductance difference
    float flux; // Flux
    float B;
    float Js; // Inertia
    float ibw; // Bandwidth
    float delta; // Delta
    float Kt; // Torque constant, equal to 1.5*NPP*Flux
    float GKt; // Gear ratio Torque constant, equal to 1.5*NPP*Flux*Gr*Gr_eff
    float div_Kt; // 1/Kt
    float div_GKt; // 1/div_GKt
    float Ke; // Back EMF constant

    float e_off;
    float r_off;
    float m_off; // Mechanical offset

    float Gr; // Gear ratio
    float Gref; // Gear efficiency
    float pn; // Pole pairs

    float pnd_2pi; // pn/2pi
    float div_Gr; // 1/Gr
    float div_pn; // 1/pn
} pmsm_para_t;

// PMSM control structure
typedef struct
{
    float drag_pe;    // Drag electrical position
    float pos_acc; // Position accuracy
    float vd_set; // d-axis voltage setpoint
    float vq_set; // q-axis voltage setpoint

    float id_set; // d-axis current setpoint
    float iq_set; // q-axis current setpoint
    float iq_lim;

    float torm_set; //
    float tor_set;

    float we_set; // electric speed

    float wr_set; // rotor speed
    float wr_lim;
    float wm_set; // mechanical speed
    float wm_ref;

    float wm_lim;
    float wm_acc; // Acceleration
    float wm_dec; // Deceleration
    float wm_diff;
    float posm_set; // Position setpoint
    float posm_ref; // Position reference
    float posr_set;

<<<<<<< HEAD
    float mit_tor_set;
    float kp;
    float kd;
} pmsm_ctrl_t;

// APP控制参数结构体
typedef struct
{
    curve_type_e curve;         // 控制曲线类型
    pos_ctrl_mode_e pos_ctrl_mode;   // 位置控制模式：绝对/相对
=======
    float wm_lst;
    float posm_lst;

    float mit_tor_set;
    float kp;
    float kd;
>>>>>>> 157aa8c814c3698051e702968095d8070515b611

    float pmax_vel;              // 正向最大速度限制
    float pmax_pos;              // 正向最大位置限制
    float pmax_tor;              // 正向最大转矩限制
<<<<<<< HEAD
=======
    float pmax_iq;               // 正向最大电流限制
    float pmax_tor_vel;          // 转矩模式正向最大速度限制
>>>>>>> 157aa8c814c3698051e702968095d8070515b611

    float nmax_vel;              // 反向最大速度限制
    float nmax_pos;              // 反向最大位置限制
    float nmax_tor;              // 反向最大转矩限制
<<<<<<< HEAD

    bool pos_reached;           // 目标位置到达信号
    bool vel_reached;           // 目标速度到达信号
    bool tor_reached;           // 目标转矩到达信号

    float wm_lst;
    float posm_lst;

    float abs_pos_ref; // 绝对位置参考值
    float rel_pos_ref; // 相对位置参考值

    bool tor_set_immediate;     // 扭矩设定是否及时更新
    bool tor_set_by_flag;       // 扭矩设定是否按标志位更新
    bool vel_set_immediate;     // 速度设定是否及时更新
    bool vel_set_by_flag;       // 速度设定是否按标志位更新
    bool pos_set_immediate;     // 位置设定是否及时更新
    bool pos_set_by_flag;       // 位置设定是否按标志位更新

} pmsm_app_ctrl_t;
=======
    float nmax_iq;               // 反向最大电流限制
    float nmax_tor_vel;          // 转矩模式反向最大速度限制
} pmsm_ctrl_t;

>>>>>>> 157aa8c814c3698051e702968095d8070515b611

// Period structure
typedef struct
{
    uint32_t start_cnt;
    uint32_t stop_cnt;
    uint32_t cur_pid_cnt;
    uint32_t spd_pid_cnt;
    uint32_t pos_pid_cnt;

    uint32_t spd_mea_cnt;
    uint32_t spd_set_cnt;

    uint32_t smo_if_cnt;

    float foc_ts;
    float foc_fs;

    float cur_pid_ts;
    float cur_pid_fs;

    float spd_pid_ts;
    float spd_pid_fs;

    float pos_pid_ts;
    float pos_pid_fs;

    float spd_mea_ts;
    float spd_mea_fs;

    float check_ts;

    uint8_t cur_pid_cnt_val;
    uint8_t spd_pid_cnt_val;
    uint8_t pos_pid_cnt_val;
    uint8_t spd_mea_cnt_val;
} period_t;

// ADC value structure for PMSM
typedef struct
{
    uint16_t ia;
    uint16_t ib;
    uint16_t ic;

    uint16_t vbus;
    uint16_t va;
    uint16_t vb;
    uint16_t vc;
    uint16_t Tmos_ab;
    uint16_t Tmos_bc;
    uint16_t Trotor;

    uint16_t sin_hall;
    uint16_t cos_hall;

    float ia_off;
    float ib_off;
    float ic_off;

} pmsm_adc_val_t;

// Board parameter structure for PMSM
typedef struct
{
    float v_ref;
    float v_adc;
    float i_res;
    float i_op;
    float i_ratio;
    float i_max;
    float rated_i;

    float v1_res;
    float v2_res;
    float v_op;
    float v_ratio;
    float v_max;
    float rated_v;

    float Rt_Mos;
    float Rt_Mos_res;
    float Rt_Mos_B;
    float Rt_Mos_Ka;

    float Rt_rotor;
    float Rt_rotor_res;
    float Rt_rotor_B;
    float Rt_rotor_Ka;

    float dead_time;
} pmsm_board_t;

<<<<<<< HEAD
// Fault status structure for PMSM
typedef union
{
    struct
    {
        uint32_t ov_curr : 1; // Overcurrent
        uint32_t un_volt : 1; // Undervoltage
        uint32_t ov_volt : 1; // Overvoltage
        uint32_t ov_tmos : 1; // Over temperature MOS
        uint32_t ov_tcoi : 1; // Over temperature coil

        uint32_t enc_err : 1; // Encoder error
        uint32_t ioff_err : 1; // Offset error
        uint32_t off_link : 1; // Link off
    } bit;

    uint32_t all;
} pmsm_fault_t;

typedef union
{
    struct
    {
        uint32_t cali_sensor1 : 1;   // 校准第一编码器
        uint32_t cali_sensor2 : 1;   // 校准第二编码器
        uint32_t cali_sensor1_done : 1; // 校准第一编码器完成标志
        uint32_t cali_sensor2_done : 1; // 校准第二编码器完成标志
        uint32_t idpm : 1;           // 电机参数辨识
        uint32_t idpm_done : 1;      // 电机参数辨识完成标志
        uint32_t fwr_pm : 1;         // 写入电机参数
        uint32_t general : 1;        // 通用标志

    } bit;

    uint32_t all;
} pmsm_flag_t;

// Protection parameter structure
typedef struct
{
    uint8_t rst;
    float uv_value;
    float ov_value;
    float oc_value;
    float ot_value;
    float omt_value;
    float lt_value;
    float time_value;

    uint32_t uv_cnt;
    uint32_t ov_cnt;
    uint32_t oc_cnt;
    uint32_t ot_cnt;
    uint32_t omt_cnt;
    uint32_t link_out_cnt;

    uint32_t uv_cnt_value;
    uint32_t ov_cnt_value;
    uint32_t oc_cnt_value;
    uint32_t ot_cnt_value;
    uint32_t omt_cnt_value;
    uint32_t link_out_cnt_value;

    float uv_time;
    float ov_time;
    float oc_time;
    float ot_time;
    float omt_time;
    float link_out_time;
} protect_t;


// Encoder parameter structure
typedef struct
{
    int dir;
    int32_t raw;
    uint8_t rev_flag;

    float pos;

    float pos_last;
    float pos_dif;

    uint32_t cpr;
    uint8_t bit;
    uint8_t shift_bit;
    float factor;

} enc_para_t;

// Calibration state enumeration
typedef enum
{
    cali_pp_start,
    cali_pp_calc,
    cali_lut_init,
    cali_lut_cw,
    cali_lut_ccw,
    cali_off_calc,
    cali_lut_calc,
    cali_lut_end
} cali_state_e;

// Calibration structure
typedef struct
{
    cali_state_e state;

    int dir;
    uint8_t pn;
    int32_t raw;
    uint8_t bit;

    float pos;

    float vd_set;
    float vd_ref;
    float vq_ref;

    float pos_err;
    float pe_set;
    float pr_set;
    float pe_acc;

    float pos_start;
    float pos_end;
    float pos_dir_end;

    uint16_t lut_num;
    uint16_t window;
    float lut[256];

    uint16_t raw_off;
    float offset;

    uint32_t n1;
    uint32_t n2;

    uint32_t i, j, k;
    float p_raw[2];
    float p_err[256];
    float mean;
    int ind;
    float p_error_arr[21 * 2 * 256];

    float e_off;
    float r_off;
} cali_t;

typedef enum
{
    mod_start,
    mod_end
} mod_state_e;

typedef struct
{
    mod_state_e state;
    uint32_t cnt;
    float vd_set;
    float vd_ref;
    float wr_set;
    float pe_acc;
    float pe_set;
    uint8_t result[2];
} mod_enc_t;

typedef struct {
	float y0;      // i(k)
	float F[2];    // [i(k-1) u(k-1)]
	float a[2 * 1];// estimation state,[1-R/L*Ts,1/L*Ts]
	float p[2 * 2];
} RLhf_t;

typedef struct {
	float y0;      // w(k)
	float F[3];    // [w(k-1)	Te(k-1)	-1]
	float a[3 * 1];// estimation state [1-B/J*T;1/J*T;Tl*T/J]
	float p[3 * 3];
} JBTl_t;

typedef struct{
	float L_est;
	float Kr;
	float Kf;
	/*input para*/
	float id_act;
	float iq_act;
	float ud;
	float uq;
	float we;
	/*est para*/
	float id_est;
	float iq_est;
	/* error */
	float ed;
	float eq;
	/* integral of product */
	float I_Rs;
	float I_Fx; // Flux
	/* transient para */
	float at;	//	R/L,		=a0-Kr*I_rs
	float bt;	//	flux/L,	=b0-Kf*I_Flux
	float ct;	//	1/L,		=1/L0
	/* initial papa */
	float a0;	//	R0/L0
	float b0;	//	Flux0/L0
	float c0;	//	1/L0
	float tp;	//	sample time
	/* output para */
	float R_est;
	float F_est;
}MRAS_Type;

// Identification state enumeration
typedef enum
{
    id_RL,
    id_Fx,
    id_JB,
} id_state_e;

// Identification state enumeration for Rs
typedef enum {
    id_RL_init,
	id_RL_align,
	id_RL_volt,
    id_RL_calc,
	id_RL_end,
} id_RL_state_e;

typedef enum
{
	id_JB_init,
	id_JB_loop,
	id_JB_calc,
	id_JB_end
} id_JB_state_e;

// Identification state enumeration for Flux
typedef enum
{
    id_Fx_init,
    id_Fx_calc,
    id_Fx_end,
} id_Fx_state_e;

// Identification state enumeration for Js
typedef enum
{
    id_Js_init,
    id_Js_loop,
    id_Js_calc,
    id_Js_end
} id_Js_state_e;

// Parameter identification structure
typedef struct
{
    float i_alph;
    float i_beta;
    float v_alph;
    float v_beta;
    float vs;

    float i_d;
    float i_q;
    float iq_f;
    float i_s;
    float v_d;
    float v_q;
    float v_s;
    float wr;
    float we;

    float vd_ref;
    float vq_ref;
    float id_ref;
    float iq_ref;

    float ts;
    float fs;
    float tc;

    // Identification Rs
    float vd_max;
    float id_min;
    float id_max;
    float vd_step;
    float RL_ts;
    float RL_fs;
    float RL_time;      // Time for Res and Ls identification
    float R_est;
    float Rs;
    float Ls;

    // Identification Flux
    float fiq_ref;
    float fkp;
    float wr_set;
    float fx_time;
    float flux;

    // Identification Js
    float JB_iq_ref;
    float JB_iq_max;

    float JB_fs;
    float JB_ts;
    float JB_time;

    float Js;
    float B;

    float Tem;
} idpm_t;

// Nonlinear observer structure
typedef struct
{
    float* i_alph; // Current in alpha-beta coordinates
    float* i_beta; // Current in alpha-beta coordinates

    float* v_alph; // Voltage in alpha-beta coordinates
    float* v_beta; // Voltage in alpha-beta coordinates

    float* i_q;

    float i_alph_lst; // Last i_alph
    float i_beta_lst; // Last i_beta
    float v_alph_lst; // Last v_alph
    float v_beta_lst; // Last v_beta

    float id_out;
    float id_ref;

    float* Rs; // Resistance of the motor
    float* Ls; // Inductance of the motor
    float* flux; // Magnetic flux
    float flux_sqr; // Square of magnetic flux
    float flux_err;

    float flux_alph; // Estimated flux in alpha axis
    float flux_beta; // Estimated flux in beta axis
    float flux_fac;
    float flux_est_s; // Square of estimated flux
    float flux_est; // Estimated total flux magnitude

    float x1; // State variables for observer
    float x2; // State variables for observer

    float id_gain;
    float gain; // Observer gain
    float bw_factor; // Bandwidth factor
    float gamma;
    float ts; // Sampling time
    float fs;

    float we;
    float pos_e; // Rotor angle

    pll_t pll;
} nlob_t;

// Adaptive linear observer structure
typedef struct
{
    float* i_alph; // Current in alpha-beta coordinates
    float* i_beta; // Current in alpha-beta coordinates

    float* v_alph; // Voltage in alpha-beta coordinates
    float* v_beta; // Voltage in alpha-beta coordinates

    float* i_q;

    float i_alph_lst; // Last i_alpha
    float i_beta_lst; // Last i_beta
    float v_alph_lst; // Last v_alpha
    float v_beta_lst; // Last v_beta

    float id_out;
    float id_ref;

    float f_valph;
    float f_vbeta;
    float f_ialph;
    float f_ibeta;
    float fd;
    float fq;
    float f_id;
    float f_iq;

    float* Rs; // Resistance of the motor
    float* Ls; // Inductance of the motor
    float* Ld; // Inductance of the motor
    float* Lq; // Inductance of the motor
    float* flux; // Magnetic flux

    float flux_alph; // Estimated flux in alpha axis
    float flux_beta; // Estimated flux in beta axis

    float esti_flux; // Estimated total flux magnitude

    float id_gain;
    float x1_gain;
    float x2_gain;
    float ts; // Sampling time
    float fs;

    float we;
    float pos_e; // Rotor angle

    pll_t pll;
} alob_t;

// Sliding mode observer structure
typedef struct
{
    float* i_alph; // Current in alpha-beta coordinates
    float* i_beta; // Current in alpha-beta coordinates

    float* v_alph; // Voltage in alpha-beta coordinates
    float* v_beta; // Voltage in alpha-beta coordinates

    float id_ref;
    float id_out;

    float i_d;
    float i_q;

    float v_d;
    float v_q;

    float e_d;
    float e_q;

    float* Rs; // Resistance of the motor
    float* Ls; // Inductance of the motor
    float* Ld; // Inductance of the motor
    float* Lq; // Inductance of the motor
    float* flux; // Magnetic flux

    float alpha0;
    float lamda1;
    float id_gain;
    float ts; // Sampling time
    float fs;

    float we;
    float pos_e; // Rotor angle
} scvm_t;

// High-frequency signal injection structure
typedef struct
{
    float iv_fs;
    float iv_ts;
    float vdh;

    float ialph_h;
    float ibeta_h;
    float ialph_f;
    float ibeta_f;

    float id_h;
    float iq_h;
    float id_f;
    float iq_f;

    float ialph_h_lst;
    float ibeta_h_lst;

    float ialph_lst;
    float ibeta_lst;
    float id_lst;
    float iq_lst;

    float dialph_h;
    float dibeta_h;

    float pos_e;

    pll_t pll;
} hfsi_t;

typedef struct
{
    float* Rs; // Resistance of the motor
    float* Ls; // Inductance of the motor
    float* Ld; // Inductance of the motor
    float* Lq; // Inductance of the motor
    float* flux; // Magnetic flux

    float* i_alph; // Current in alpha-beta coordinates
    float* i_beta; // Current in alpha-beta coordinates
    float* v_alph; // Voltage in alpha-beta coordinates
    float* v_beta; // Voltage in alpha-beta coordinates
    float* we;

    float Ealph; // Variable: Stationary alfa-axis back EMF
    float Zalph; // Output: Stationary alfa-axis sliding control
    float Gsmopos; // Parameter: mt dependent control gain
    float EstIalph; // Variable: Estimated stationary alfa-axis stator current
    float Fsmopos; // Parameter: mt dependent plant matrix
    float Ebeta; // Variable: Stationary beta-axis back EMF
    float Zbeta; // Output: Stationary beta-axis sliding control
    float EstIbeta; // Variable: Estimated stationary beta-axis stator current
    float IalphError; // Variable: Stationary alfa-axis current error
    float Kslide; // Parameter: Sliding control gain
    float IbetaError; // Variable: Stationary beta-axis current error
    float Kslf; // Parameter: Sliding control filter gain
    float E0; // Parameter: 0.5

    float m_theta;

    float pos_e;
    float wc;

    float we_est;

    float ts;
    float fs;
} esmo_t;

// Initial position detection structure
typedef struct
{
    u8 mode;
    u8 step;
    u8 section;
    u8 over_flag;
    f32 init_pos;
    f32 is_max;
    f32 is[12];
} ipd_t;

typedef struct
{
    // Step
    float pos_step;	// Current pos at the given tick
    float spd_step; // Current speed at the given tick
    float tor_step; // Current torque at the given tick

    float pos_init;
    float pos_end;

    float vel_init;
    float vel_end;

    float acc;
    float vel;
    float dec;

    float acc_distance;

    float t_acc;
    float t_vel;
    float t_dec;
    float t_total;

    uint32_t tick;
    curve_type_e curve_mode;
    bool profile_done;
    uint8_t flag;
    float ts;

} traj_t;

typedef struct
{
    float ts;
    float wn;
    float g1,g2,g3;
    float wkbf1, wkbf2;
    float acc;

    float kj;
    float tf;
    float wr_obs;
    float pr_obs;
} eh_vobs_t;

typedef struct
{
    float ts;
    float g1,g2;
    float a[2][2];
    float b[2];
    float c[2];
    float z[2];
    float x[2];
    float u;

    float z1_dot;
    float z2_dot;
    float we_obs;
    float Tl_obs;
} eh_tobs_t;


typedef struct
{
    pos_mode_e     pos_mode;   // 位置模式：有感/无感
    senless_type_e senless;    // 无感类型
    encoder_type_e sensory1;    // 主编码器类型
    encoder_type_e sensory2;   // 第二编码器类型（如有）

    // 编码器参数
    enc_para_t ma732;
    enc_para_t mt6816;
    enc_para_t mt6825;
    enc_para_t dm485enc;

    int32_t raw_1;
    uint8_t bit_1;
    float pos_1;

    int32_t raw_2;
    uint8_t bit_2;
    float pos_2;
} pos_box_t;

=======
>>>>>>> 157aa8c814c3698051e702968095d8070515b611

// PMSM structure
typedef struct
{
<<<<<<< HEAD
    mode_ctrl_t  mode;
    pm_ctrl_bit_e ctrl_bit;
    pm_state_bit_e state_bit;

=======
    pm_ctrl_bit_e ctrl_bit;
    pm_state_bit_e state_bit;
>>>>>>> 157aa8c814c3698051e702968095d8070515b611
    pmsm_board_t board;
    pmsm_adc_val_t adc;
    pmsm_para_t para;
    pmsm_ctrl_t ctrl;
<<<<<<< HEAD
    pmsm_app_ctrl_t app_ctrl;
    pmsm_foc_t foc;
    period_t period;
    pmsm_fault_t fault;
    pmsm_flag_t flag;
    protect_t protect;

    pos_box_t pos_box;

    cali_t calibr;
    mod_enc_t modenc;
    idpm_t idpm;

    nlob_t nlob;
    alob_t alob;
    scvm_t scvm;
    esmo_t esmo;
    hfsi_t hfsi;

    pid_para_t vq_pi;
    pid_para_t id_pi;
    pid_para_t iq_pi;
    pid_para_t spd_pi;
    pid_para_t pos_pi;

    lpf_t id_lpf;
    lpf_t iq_lpf;
    lpf_t vd_lpf;
    lpf_t vq_lpf;
    lpf_t ibus_lpf;
    lpf_t vbus_lpf;
    lpf_t iabs_lpf;
    lpf_t wr_lpf;

    traj_t traj;
=======
    pmsm_foc_t foc;
    period_t period;

>>>>>>> 157aa8c814c3698051e702968095d8070515b611
} pmsm_t;

extern pmsm_t pm;

<<<<<<< HEAD
extern eh_vobs_t eh_vobs;
extern eh_tobs_t eh_tobs;

void temp_calc(void);

/* Utility functions */
float sat1_datf(float val, float up, float low);
float fast_atan2(float y, float x);
void low_pf_init(lpf_t* x);
float low_pf(lpf_t* x, float val);
float sin_f32(float x);
float cos_f32(float x);

uint8_t crc8(const uint8_t* data, const uint32_t size);
uint32_t crc32(const uint8_t* data, uint32_t size);

int uint32_to_data(uint32_t val, uint8_t* data);
int int32_to_data(int32_t val, uint8_t* data);
int uint16_to_data(uint16_t val, uint8_t* data);
int int16_to_data(int16_t val, uint8_t* data);
int float_to_data(float val, uint8_t* data);

uint32_t data_to_uint32(uint8_t* data);
int32_t data_to_int32(uint8_t* data);
uint16_t data_to_uint16(uint8_t* data);
int16_t data_to_int16(uint8_t* data);
float data_to_float(uint8_t* data);

/* PID functions */
void pid_para_init(pid_para_t* pid_config);
void pid_limit_init(pid_para_t* pid_config, float i_term_max, float i_term_min, float out_max, float out_min);
void pid_clear(pid_para_t* pid_clear);
void pid_reset(pid_para_t* pid_config, float kp, float ki, float kd);
float parallel_pid_ctrl(pid_para_t *pi, float ref_value, float fback_value);
float serial_pid_ctrl(pid_para_t* pid, float ref_value, float fdback_value);
float serial_pid_ctrl1(pid_para_t* pid, float ref_value, float fdback_value, float i_max, float out_max);
float pdff_ctrl(pid_para_t *pid, float ref_value, float fdback_value);

/* PLL functions */
void pll_calc(pll_t* pll, float pos);
void ort_pll_calc(pll_t* pll, float alpha, float beta, float gain);

=======
>>>>>>> 157aa8c814c3698051e702968095d8070515b611
/* FOC calculation functions */
void foc_calc(pmsm_foc_t * foc);
void sin_cos_val(pmsm_foc_t* foc);
void clarke_transform(pmsm_foc_t* foc);
void inverse_clarke(pmsm_foc_t* foc);
void park_transform(pmsm_foc_t* foc);
void inverse_park(pmsm_foc_t* foc);
void svpwm_midpoint(pmsm_foc_t* foc);
void svpwm_sector(pmsm_foc_t* foc);
int svpwm(pmsm_foc_t* foc);
int svm(float alpha, float beta, float* ta, float* tb, float* tc);

/* FOC drive functions */
void pmsm_init(void);
<<<<<<< HEAD
=======
void pmsm_peroid_init(void);
void pmsm_protect_init(void);
>>>>>>> 157aa8c814c3698051e702968095d8070515b611
void pmsm_lpf_init(void);
void foc_para_calc(pmsm_t* pm);
void foc_pwm_start(void);
void foc_pwm_stop(void);
<<<<<<< HEAD
void foc_clear(void);
void foc_pwm_run(pmsm_t* pm);
void foc_pwm_duty_set(pmsm_t* pm);
void foc_adc_sample(pmsm_t* pm);
void foc_get_curr_off(void);
=======
void foc_clear(pmsm_t* pm);
void foc_pwm_run(pmsm_t* pm);
void foc_pwm_duty_set(pmsm_t* pm);
void foc_adc_sample(pmsm_t* pm);
void get_curr_off(void);
>>>>>>> 157aa8c814c3698051e702968095d8070515b611
float foc_spd_measure_M(float pos, float fs);

void foc_cur_pi_calc(pmsm_t* pm);
void foc_spd_pi_calc(pmsm_t* pm);

void foc_volt(pmsm_t* pm, float vd_ref, float vq_ref, float pos);
<<<<<<< HEAD
void foc_curr(pmsm_t* pm, float id_set, float iq_set, float pos);
void foc_vel(pmsm_t* pm, float vel_set, float iq_set, float pos);
void foc_pos(pmsm_t* pm, float pos_set, float vel_set, float iq_set, float pos);
=======
>>>>>>> 157aa8c814c3698051e702968095d8070515b611

/* FOC control functions */
void pmsm_state_ctrl(pmsm_t* pm);
void pmsm_mode_ctrl(pmsm_t* pm);
<<<<<<< HEAD
void pmsm_observe(pmsm_t* pm);
void pmsm_fault_check(pmsm_t* pm);

void force_volt_mode(pmsm_t* pm);
void open_volt_mode(pmsm_t* pm);
void force_curr_mode(pmsm_t* pm);

void sensory_pos_calc(pmsm_t* pm);
void senless_pos_calc(pmsm_t* pm);
void pos_calc(pmsm_t* pm);

void axdr_mit_mode(pmsm_t* pm);
void axdr_tor_mode(pmsm_t* pm);
void axdr_vel_mode(pmsm_t* pm);
void axdr_pos_mode(pmsm_t* pm);

/* Encoder functions */
void encoder_init(void);
uint32_t read_mt6825_raw(void);
uint32_t read_mt6816_raw(void);
uint32_t read_ma732_raw(void);
uint32_t read_dm485enc_raw(void);
uint32_t send_mod_dm485enc(void);

void pos_calc(pmsm_t* pm);
void send_encoder_read_command(pmsm_t* pm);
void pos_encoder_calc(enc_para_t* x, pmsm_t* pm);
void sensory1_pos_calc(pmsm_t* pm);
void sensory2_pos_calc(pmsm_t* pm);
void senless_pos_calc(pmsm_t* pm);
float spd_measure(float pos, float fs, float filt_bw);

/* PMSM identification functions */
void iden_init(void);
void iden_pmsm(idpm_t *x);
void iden_RL(idpm_t *obj);
void iden_JB(idpm_t *obj);
void iden_Fx(idpm_t *obj);


/* Calibration functions */
void cali_init(void);
void cali_mag_encoder(pmsm_t *pm);
void modulation_encoder(pmsm_t *pm);

/* Nonlinear observer functions */
void nlob_init(void);
void nlob_vesc(nlob_t* obj);

/* Adaptive linear observer functions */
void alob_init(void);
void alob_flux(alob_t* obj);
void alob_curr(alob_t* obj);

/* Execute the static compensation voltage functions */
void scvm_init(void);
void scvm_obe(scvm_t* obj);

/* High-frequency signal injection functions */
void hfsi_init(void);
void hfsi_input(void);

/* E smo */
void esmo_init(void);
void esmo_obe(esmo_t* x);

void vofa_start(void);

/* Initial position detection functions */
void ipd_pos_run(ipd_t* obj, float* vd);

void traj_init(void);
//void spd_traj_plan(traj_t *x, float vel_init, float vel_end, float acc, float dec, float curve_mode);
//void spd_traj_eval(traj_t *x);
//
//void pos_traj_plan(traj_t *x, float pos_init, float pos_end, float vel_init, float v_max, float acc, float dec, float curve_mode);
//void pos_traj_eval(traj_t *x);
extern traj_t Traj;
void spd_traj_plan(float vel_init, float vel_end, float acc, float dec, float curve_mode);
void spd_traj_eval(void);

void pos_traj_plan(float pos_init, float pos_end, float vel_init, float v_max, float acc, float dec, float curve_mode);
void pos_traj_eval(void);

void upd_RLHF(RLhf_t *v);
void upd_JBTl(JBTl_t *h);
void upd_Flux(MRAS_Type *x);

void eh_observer_init(void);
void eh_speed_observer(eh_vobs_t *x, float wr, float tor);
void eh_torque_observer(eh_tobs_t *x, float we, float tor);
=======

void force_volt_mode(pmsm_t* pm);
>>>>>>> 157aa8c814c3698051e702968095d8070515b611

#endif
