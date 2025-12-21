#include "common.h"

_RAM_FUNC void pll_calc(pll_t *pll, float pos)
{

	pll->err = pos - pll->pos_e;
	
	wrap_pm_pi(pll->err);
	
	pll->p_term = pll->kp*pll->err;
	pll->i_term += pll->ki*pll->err*pll->ts;
	
	if (pll->i_term > pll->i_term_max)
		pll->i_term = pll->i_term_max;
	else if (pll->i_term < -pll->i_term_max)
		pll->i_term = -pll->i_term_max;
	
	pll->out_value = pll->p_term + pll->i_term;
	
	if (pll->out_value > pll->out_max)
		pll->out_value = pll->out_max;
	else if (pll->out_value < -pll->out_max)
		pll->out_value = -pll->out_max;
	
	pll->we = pll->out_value;
	pll->pos_e += pll->we*pll->ts;
	wrap_0_2pi(pll->pos_e);
}

_RAM_FUNC void ort_pll_calc(pll_t *pll, float alpha, float beta, float gain)
{
	pll->err = (beta*pll->cos_val - alpha*pll->sin_val)/gain;
	
	pll->p_term = pll->kp*pll->err;
    pll->i_term += pll->err*pll->ki*pll->ts;
	
	if (pll->i_term > pll->i_term_max)
		pll->i_term = pll->i_term_max;
	else if (pll->i_term < -pll->i_term_max)
		pll->i_term = -pll->i_term_max;
	
	pll->out_value = pll->p_term + pll->i_term;
	
	if (pll->out_value > pll->out_max)
		pll->out_value = pll->out_max;
	else if (pll->out_value < -pll->out_max)
		pll->out_value = -pll->out_max;
	
	pll->we = pll->out_value;
    pll->pos_e += pll->we*pll->ts;

    pll->sin_val = sin_f32(pll->pos_e);
    pll->cos_val = cos_f32(pll->pos_e);
	
	wrap_0_2pi(pll->pos_e);
}




