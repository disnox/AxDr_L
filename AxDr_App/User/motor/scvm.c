#include "common.h"

void scvm_init(void)
{
	memset(&pm.scvm, 0, sizeof(pm.scvm));

	pm.scvm.v_alph = &pm.foc.v_alph;
	pm.scvm.v_beta = &pm.foc.v_beta;
	pm.scvm.i_alph = &pm.foc.i_alph;
	pm.scvm.i_beta = &pm.foc.i_beta;

	pm.scvm.Rs = &pm.para.Rs;
	pm.scvm.Ls = &pm.para.Ls;
	pm.scvm.Ld = &pm.para.Ld;
	pm.scvm.Lq = &pm.para.Lq;
	pm.scvm.flux = &pm.para.flux;

	pm.scvm.alpha0 = 1500;
	pm.scvm.lamda1 = 0.99f;
	pm.scvm.id_gain = 0.8f; //0-2之间 越小越稳定，越大低速性能越强
	pm.scvm.ts = 0.00005f;
	pm.scvm.fs = 20000;
}


_RAM_FUNC void scvm_obe(scvm_t *obj)
{
	float sign_we = obj->we > 0 ? 1 : -1; 
    float abs_wc = fabsf(obj->we);
	
	if(abs_wc > 500)
    {
        obj->id_out = 0;
    }
    else
    {
        obj->id_ref = obj->i_q*obj->id_gain*sign_we;
        obj->id_out = obj->id_out * 0.999f + obj->id_ref * 0.001f;
    }
	
	float sin_val = sin_f32(obj->pos_e);
    float cos_val = cos_f32(obj->pos_e);
	
	obj->e_d = obj->v_d - obj->Rs[0]*obj->i_d + obj->we*obj->Ls[0]*obj->i_q;
    obj->e_q = obj->v_q - obj->Rs[0]*obj->i_q - obj->we*obj->Ls[0]*obj->i_d;
	
	float alpha = obj->alpha0 + 2.0f*obj->lamda1 * abs_wc;
    float wr = (obj->e_q - obj->e_d*sign_we*obj->lamda1)/obj->flux[0];
    float delta_w = alpha*(wr - obj->we);

    obj->we += delta_w*obj->ts;
    obj->pos_e += obj->we*obj->ts;
	wrap_0_2pi(obj->pos_e);
	
	// calc park
    obj->v_d = cos_val*obj->v_alph[0] + sin_val*obj->v_beta[0];
    obj->v_q = cos_val*obj->v_beta[0] - sin_val*obj->v_alph[0];
	
	// calc park
    obj->i_d = cos_val*obj->i_alph[0] + sin_val*obj->i_beta[0];
    obj->i_q = cos_val*obj->i_beta[0] - sin_val*obj->i_alph[0];
	
}



