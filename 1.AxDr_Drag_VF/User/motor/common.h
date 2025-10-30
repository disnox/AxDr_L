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
#define M_2_PI (6.28318530716f)        // 2 * Pi
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

#define Dead_Time 80
#define PWM_ARR() __HAL_TIM_GET_AUTORELOAD(&htim1)

#define set_dtc_a(value) __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, value)
#define set_dtc_b(value) __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, value)
#define set_dtc_c(value) __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, value)

#define cs_down       HAL_GPIO_WritePin(SPI1_CSN_GPIO_Port, SPI1_CSN_Pin, GPIO_PIN_RESET);
#define cs_up         HAL_GPIO_WritePin(SPI1_CSN_GPIO_Port, SPI1_CSN_Pin, GPIO_PIN_SET);


typedef enum
{
	foc_volt_mode,
	foc_curr_mode,
	foc_vel_mode,
	foc_pos_mode,

} foc_mode_e;

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
    fault = 3,
} pm_state_bit_e;

// PMSM parameter structure
typedef struct
{
    float rated_voltage;   // 额定电压
    float rated_current;   // 额定母线电流
    float rated_speed;     // 额定转速
    float rated_torque;    // 额定扭矩
    float rated_power;     // 额定功率
    float peak_current;    // 峰值母线电流
    float peak_torque;     // 峰值扭矩
    float peak_speed;      // 峰值转速
	
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

    float wm_lst;
    float posm_lst;

    float mit_tor_set;
    float kp;
    float kd;

    float pmax_vel;              // 正向最大速度限制
    float pmax_pos;              // 正向最大位置限制
    float pmax_tor;              // 正向最大转矩限制
    float pmax_iq;               // 正向最大电流限制
    float pmax_tor_vel;          // 转矩模式正向最大速度限制

    float nmax_vel;              // 反向最大速度限制
    float nmax_pos;              // 反向最大位置限制
    float nmax_tor;              // 反向最大转矩限制
    float nmax_iq;               // 反向最大电流限制
    float nmax_tor_vel;          // 转矩模式反向最大速度限制
} pmsm_ctrl_t;


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


// PMSM structure
typedef struct
{
    pm_ctrl_bit_e ctrl_bit;
    pm_state_bit_e state_bit;
    pmsm_board_t board;
    pmsm_adc_val_t adc;
    pmsm_para_t para;
    pmsm_ctrl_t ctrl;
    pmsm_foc_t foc;
    period_t period;

} pmsm_t;

extern pmsm_t pm;

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
void pmsm_peroid_init(void);
void pmsm_protect_init(void);
void pmsm_lpf_init(void);
void foc_para_calc(pmsm_t* pm);
void foc_pwm_start(void);
void foc_pwm_stop(void);
void foc_clear(pmsm_t* pm);
void foc_pwm_run(pmsm_t* pm);
void foc_pwm_duty_set(pmsm_t* pm);
void foc_adc_sample(pmsm_t* pm);
void get_curr_off(void);
float foc_spd_measure_M(float pos, float fs);

void foc_cur_pi_calc(pmsm_t* pm);
void foc_spd_pi_calc(pmsm_t* pm);

void foc_volt(pmsm_t* pm, float vd_ref, float vq_ref, float pos);

/* FOC control functions */
void pmsm_state_ctrl(pmsm_t* pm);
void pmsm_mode_ctrl(pmsm_t* pm);

void force_volt_mode(pmsm_t* pm);

#endif
