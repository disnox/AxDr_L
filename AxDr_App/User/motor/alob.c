#include "common.h"

alob_t alob;

void alob_init(void)
{
    // Initialize the active linear observer structure
    memset(&alob, 0, sizeof(alob));
    
    // Assign pointers to the FOC parameters
    alob.v_alph = &pm.foc.v_alph;
    alob.v_beta = &pm.foc.v_beta;
    alob.i_alph = &pm.foc.i_alph;
    alob.i_beta = &pm.foc.i_beta;
    alob.i_q    = &pm.foc.i_q;
    
    // Assign pointers to the motor parameters
    alob.Rs = &pm.para.Rs;
    alob.Ls = &pm.para.Ls;
    alob.Ld = &pm.para.Ls;
    alob.Lq = &pm.para.Ls;
    alob.flux = &pm.para.flux;
    
    // Set observer gains
    alob.id_gain = 0.1f;
    alob.x1_gain = 1.3f; // Gain for state variable x1
    alob.x2_gain = 1.3f; // Gain for state variable x2
    
    // Set sampling time and frequency
    alob.ts = 0.00005f;
    alob.fs = 20000;
    
    // Initialize flux values
    alob.flux_alph = alob.flux[0];
    alob.flux_beta = 0;
    
    // Initialize PLL parameters
    alob.pll.wn   = 100 * 2 * M_PI;  // Natural frequency
    alob.pll.damp = 0.707f;          // Damping ratio
    alob.pll.ts   = 0.00005f;
    
    // Calculate PLL gains
    alob.pll.kp = 2 * alob.pll.damp * alob.pll.wn;
    alob.pll.ki = alob.pll.wn * alob.pll.wn;
    alob.pll.i_term_max = 333 * 2 * M_PI * 2;
    alob.pll.out_max = 333 * 2 * M_PI * 2;
}

_RAM_FUNC void alob_flux(alob_t *obj)
{
    // Determine the sign of the electrical angular velocity
    float sign_we = obj->we > 0 ? 1 : -1; 
    float abs_wc = fabsf(obj->we);
    
    // If the angular velocity is too high, reset id_out
    if(abs_wc > 1000)
    {
        obj->id_out = 0;
    }
    else
    {
        // Calculate reference current and update id_out
        obj->id_ref = obj->i_q[0] * obj->id_gain * sign_we;
        obj->id_out = obj->id_out * 0.999f + obj->id_ref * 0.001f;
    }
    
    // Calculate resistive and inductive components
    float R_ia = obj->Rs[0] * obj->i_alph_lst;
    float L_ia = obj->Ls[0] * obj->i_alph[0];
    
    float R_ib = obj->Rs[0] * obj->i_beta_lst;
    float L_ib = obj->Ls[0] * obj->i_beta[0];

    // Calculate sine and cosine of the electrical position
    float sin_val = sin_f32(obj->pos_e);
    float cos_val = cos_f32(obj->pos_e);
    
    // Calculate Park transformation
    obj->fd = cos_val * obj->f_valph + sin_val * obj->f_vbeta;
    obj->fq = cos_val * obj->f_vbeta - sin_val * obj->f_valph;
    
    // Calculate dq currents
    obj->f_id = (obj->fd - obj->flux[0]) / obj->Ld[0];
    obj->f_iq = obj->fq / obj->Lq[0];
    
    // Calculate Clarke transformation
    obj->f_ialph = cos_val * obj->f_id - sin_val * obj->f_iq;
    obj->f_ibeta = sin_val * obj->f_id + cos_val * obj->f_iq;
    
    // Calculate errors
    float x1_err = obj->i_alph[0] - obj->f_ialph;
    float x2_err = obj->i_beta[0] - obj->f_ibeta;
    
    // Calculate derivatives
    float x1_dot = obj->v_alph[0] - R_ia;
    float x2_dot = obj->v_beta[0] - R_ib;
    
    // Update state variables
    obj->f_valph += obj->ts * (x1_dot + x1_err * obj->x1_gain);
    obj->f_vbeta += obj->ts * (x2_dot + x2_err * obj->x2_gain);

    // Update flux estimates
    obj->flux_alph = obj->f_valph - L_ia;
    obj->flux_beta = obj->f_vbeta - L_ib;

#if 1
    // Calculate electrical position using atan2
    obj->pos_e = atan2f(obj->flux_beta, obj->flux_alph);
    wrap_0_2pi(obj->pos_e);
    pll_calc(&obj->pll, obj->pos_e);
#endif

#if 0
    /* Orthogonal PLL */
    ort_pll_calc(&obj->pll, obj->flux_alpha, obj->flux_beta, obj->flux);
    obj->pos_e = obj->pll.pos_e;
#endif
    
    // Store the last values of i_alpha, i_beta, v_alpha, v_beta
    obj->i_alph_lst = obj->i_alph[0];
    obj->i_beta_lst = obj->i_beta[0];
    obj->v_alph_lst = obj->v_alph[0];
    obj->v_beta_lst = obj->v_beta[0];
}

void alob_curr(alob_t *obj)
{
    // Determine the sign of the electrical angular velocity
    float sign_we = obj->we > 0 ? 1 : -1; 
    float abs_wc = fabs(obj->we);
    
    // If the angular velocity is too high, reset id_out
    if(abs_wc > 500)
    {
        obj->id_out = 0;
    }
    else
    {
        // Calculate reference current and update id_out
        obj->id_ref = obj->i_q[0] * obj->id_gain * sign_we;
        obj->id_out = obj->id_out * 0.999f + obj->id_ref * 0.001f;
    }
    
    // Calculate resistive and inductive components
    float R_ia = obj->Rs[0] * obj->i_alph_lst;
    float L_ia = obj->Ls[0] * obj->i_alph[0];
    
    float R_ib = obj->Rs[0] * obj->i_beta_lst;
    float L_ib = obj->Ls[0] * obj->i_beta[0];

    // Calculate sine and cosine of the electrical position
    float sin_val = sin_f32(obj->pos_e);
    float cos_val = cos_f32(obj->pos_e);
    
    // Calculate Park transformation
    obj->f_id = cos_val * obj->i_alph[0] + sin_val * obj->i_beta[0];
    obj->f_iq = cos_val * obj->i_beta[0] - sin_val * obj->i_alph[0];
    
    // Calculate dq currents
    obj->fd = obj->f_id * obj->Ld[0] + obj->flux[0];
    obj->fq = obj->f_iq * obj->Lq[0];
    
    // Calculate Clarke transformation
    obj->f_ialph = cos_val * obj->fd - sin_val * obj->fq;
    obj->f_ibeta = sin_val * obj->fd + cos_val * obj->fq;
    
    // Calculate errors
    float x1_err = obj->f_ialph - obj->f_valph;
    float x2_err = obj->f_ibeta - obj->f_vbeta;
    
    // Calculate derivatives
    float x1_dot = obj->v_alph[0] - R_ia;
    float x2_dot = obj->v_beta[0] - R_ib;
    
    // Update state variables
    obj->f_valph += obj->ts * (x1_dot + x1_err * obj->x1_gain);
    obj->f_vbeta += obj->ts * (x2_dot + x2_err * obj->x2_gain);

    // Update flux estimates
    obj->flux_alph = obj->f_valph - L_ia;
    obj->flux_beta = obj->f_vbeta - L_ib;

#if 1
    // Calculate electrical position using atan2
    obj->pos_e = atan2f(obj->flux_beta, obj->flux_alph);
    wrap_0_2pi(obj->pos_e);
    pll_calc(&obj->pll, obj->pos_e);
#endif

#if 0
    /* Orthogonal PLL */
    ort_pll_calc(&obj->pll, obj->flux_alpha, obj->flux_beta, obj->flux);
    obj->pos_e = obj->pll.pos_e;
#endif
    
    // Store the last values of i_alpha, i_beta, v_alpha, v_beta
    obj->i_alph_lst = obj->i_alph[0];
    obj->i_beta_lst = obj->i_beta[0];
    obj->v_alph_lst = obj->v_alph[0];
    obj->v_beta_lst = obj->v_beta[0];
}