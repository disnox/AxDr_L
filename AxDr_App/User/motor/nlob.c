#include "common.h"


void nlob_init(void)
{
	memset(&pm.nlob, 0, sizeof(pm.nlob));
	
	pm.nlob.i_alph = &pm.foc.i_alph;
	pm.nlob.i_beta = &pm.foc.i_beta;
	
	pm.nlob.v_alph = &pm.foc.v_alph;
	pm.nlob.v_beta = &pm.foc.v_beta;

	pm.nlob.i_q  = &pm.foc.i_q;
	
	pm.nlob.Rs = &pm.para.Rs;
	pm.nlob.Ls = &pm.para.Ls;
	pm.nlob.flux = &pm.para.flux;
	
	pm.nlob.flux_sqr  = pm.nlob.flux[0] * pm.nlob.flux[0];
	pm.nlob.bw_factor = 1.0f / pm.nlob.flux_sqr;
	pm.nlob.gain  = 1000.0f;
	pm.nlob.gamma = 0.5f * (pm.nlob.gain * pm.nlob.bw_factor);
	pm.nlob.id_gain = 1.0f;
	pm.nlob.ts = 0.00005f;
	pm.nlob.fs = 20000;
	
	pm.nlob.x1 = pm.nlob.flux[0];
	pm.nlob.x2 = 0;
	
	pm.nlob.pll.wn   = 100*2*M_PI;		// 带宽
	pm.nlob.pll.damp = 0.707f; 		// 阻尼系数
	pm.nlob.pll.ts   = 0.00005f;
	
	pm.nlob.pll.kp = 2*pm.nlob.pll.damp*pm.nlob.pll.wn;
	pm.nlob.pll.ki = pm.nlob.pll.wn*pm.nlob.pll.wn;
	pm.nlob.pll.i_term_max = 333*2*M_PI*2;
	pm.nlob.pll.out_max = 333*2*M_PI*2;
}



_RAM_FUNC void nlob_vesc(nlob_t *obj)
{	
	float sign_we = obj->we > 0 ? 1 : -1; 
    float abs_wc = fabsf(obj->we);
	
	if(abs_wc > 500)
    {
        obj->id_out = 0;
    }
    else
    {
        obj->id_ref = obj->i_q[0]*obj->id_gain*sign_we;
        obj->id_out = 1.5f;//fabsf(obj->id_out * 0.999f + obj->id_ref * 0.001f);
    }
	
	
	float R_ia = obj->Rs[0] * obj->i_alph_lst;
	float L_ia = obj->Ls[0] * obj->i_alph[0];
	float L_ia_prev = obj->Ls[0] * obj->i_alph_lst;
	
	float R_ib = obj->Rs[0] * obj->i_beta_lst;
	float L_ib = obj->Ls[0] * obj->i_beta[0];
	float L_ib_prev = obj->Ls[0] * obj->i_beta_lst;
	
	float err = obj->flux_sqr - obj->flux_est_s;
	float x1_dot =  obj->v_alph_lst - R_ia +  obj->gamma * obj->flux_alph * err;
	float x2_dot =  obj->v_beta_lst - R_ib +  obj->gamma * obj->flux_beta * err;
	
	obj->x1 += x1_dot * obj->ts;
	obj->x2 += x2_dot * obj->ts;

	obj->flux_alph = obj->x1 - L_ia;
	obj->flux_beta = obj->x2 - L_ib;
	obj->flux_est_s= SQ(obj->flux_alph) + SQ(obj->flux_beta);
	obj->flux_est  = sqrtf(obj->flux_est_s);

	obj->flux_err = *obj->flux - obj->flux_est;

#if 1
	// atan
	obj->pos_e = atan2f(obj->flux_beta, obj->flux_alph);
	wrap_0_2pi(obj->pos_e);
	pll_calc(&obj->pll, obj->pos_e);
#endif

#if 0
	/* Orthogonal PLL */
	ort_pll_calc(&obj->pll, obj->flux_alph, obj->flux_beta, obj->flux[0]);
#endif
	
	// 更新上一次的 i_alpha, i_beta, v_alpha, v_beta
    obj->i_alph_lst = obj->i_alph[0];
    obj->i_beta_lst = obj->i_beta[0];
    obj->v_alph_lst = obj->v_alph[0];
    obj->v_beta_lst = obj->v_beta[0];
}



