#include "common.h"

hfsi_t hfsi;

float SQ_HFI_POLE_INJ_VOLT;
int SQ_HFI_POLE_CNT;
int SQ_HFI_POLE_JUDGE_FLAG;
float SQ_HFI_POLE_0_SUM_1;
float SQ_HFI_POLE_180_SUM_1;
float SQ_HFI_POLE_0_SUM_2;
float SQ_HFI_POLE_180_SUM_2;
float SQ_HFI_POLE_0_SUM_3;
float SQ_HFI_POLE_180_SUM_3;
int SQ_HFI_POLE_JUDGE_CNT;
int8_t SQ_HFI_POLE_ERR_FALG = 0.0f;
float SQHFI_POLE_ERR_OFFSTE_ANGLE = 0.0f;
int SQ_HFI_POLE_TIME = 15;
float SQHFI_0_SPEED_OFFSET = 0.0f;
float SQ_HFI_POLE_INJ_VOLT_UP;
float SQ_HFI_POLE_INJ_VOLT_DOWN;
int SQ_HFI_POLE_START_CNT;

float Ialph_L,Ibeta_L;

void hfsi_init(void)
{
	hfsi.vdh =  2.0f;
	
	hfsi.pll.wn   = 100*2*M_PI;		// 带宽
	hfsi.pll.damp = 0.707f; 		// 阻尼系数
	hfsi.pll.ts   = 0.00005f;
	
	hfsi.pll.kp = 2*hfsi.pll.damp*hfsi.pll.wn;
	hfsi.pll.ki = hfsi.pll.wn*hfsi.pll.wn;
	hfsi.pll.i_term_max = 333*2*M_PI*2;
	hfsi.pll.out_max = 333*2*M_PI*2;

}

_RAM_FUNC void hfsi_input(void)
{
//	if(++hfsi.count == 1)
//	{
//		hfsi.count = 0;
//		hfsi.polar *= -1;
		hfsi.vdh = hfsi.vdh * -1.0f;
//  }
	hfsi.ialph_h = (pm.foc.v_alph - hfsi.ialph_lst)*0.5f;
	hfsi.ibeta_h = (pm.foc.v_beta - hfsi.ibeta_lst)*0.5f;
	
	hfsi.dialph_h = hfsi.ialph_h - hfsi.ialph_h_lst;
	hfsi.dibeta_h = hfsi.ibeta_h - hfsi.ibeta_h_lst;
	
	hfsi.id_f = (pm.foc.i_d + hfsi.id_lst)*0.5f;
	hfsi.iq_f = (pm.foc.i_q + hfsi.iq_lst)*0.5f;
	
	//极性辨识
	if(SQ_HFI_POLE_START_CNT != 10000)
	{
		SQ_HFI_POLE_START_CNT++;
	}
	else if(SQ_HFI_POLE_START_CNT == 10000)
	{
		SQ_HFI_POLE_JUDGE_FLAG = 1;
	}
	#if 1       //一次判断
	if(SQ_HFI_POLE_JUDGE_FLAG == 1)
	{
		Ialph_L = (pm.foc.i_alph + hfsi.ialph_lst)*0.5f;
		Ibeta_L = (pm.foc.i_beta + hfsi.ibeta_lst)*0.5f;
		SQ_HFI_POLE_CNT++;
		if(SQ_HFI_POLE_CNT>0 && SQ_HFI_POLE_CNT<=(SQ_HFI_POLE_TIME-5))
		{
			SQ_HFI_POLE_INJ_VOLT = 3.0f;
			if(Ialph_L<0)
			{
				Ialph_L = -Ialph_L;
			}
			if(Ibeta_L<0)
			{
				Ibeta_L = -Ibeta_L;
			}
			SQ_HFI_POLE_0_SUM_1 += (Ialph_L + Ibeta_L);
		}
		else if(SQ_HFI_POLE_CNT>(SQ_HFI_POLE_TIME-5) && SQ_HFI_POLE_CNT<(SQ_HFI_POLE_TIME+5))
		{
			SQ_HFI_POLE_INJ_VOLT = 0.00f;
		}
		else if(SQ_HFI_POLE_CNT>(SQ_HFI_POLE_TIME+5) && SQ_HFI_POLE_CNT<=SQ_HFI_POLE_TIME*2)
		{
			SQ_HFI_POLE_INJ_VOLT = -3.0f;
			if(Ialph_L<0)
			{
				Ialph_L = -Ialph_L;
			}
			if(Ibeta_L<0)
			{
				Ibeta_L = -Ibeta_L;
			}
			SQ_HFI_POLE_180_SUM_1 += (Ialph_L + Ibeta_L);
		}
		else if(SQ_HFI_POLE_CNT==SQ_HFI_POLE_TIME*2+1)
		{
			if(SQ_HFI_POLE_0_SUM_1>SQ_HFI_POLE_180_SUM_1)
			{
				SQ_HFI_POLE_ERR_FALG = 0;
				SQHFI_POLE_ERR_OFFSTE_ANGLE = 0.0f;
			}
			else if(SQ_HFI_POLE_0_SUM_1<SQ_HFI_POLE_180_SUM_1)
			{
				SQ_HFI_POLE_ERR_FALG = 1;
				SQHFI_POLE_ERR_OFFSTE_ANGLE = M_PI;
			}
				SQ_HFI_POLE_INJ_VOLT = 0.0f;
				SQ_HFI_POLE_JUDGE_FLAG = 0;
		}
//		else if(SQ_HFI_POLE_CNT==SQ_HFI_POLE_TIME*3)
//		{
//			SQ_HFI_POLE_INJ_VOLT = -2.0f;
//		}
//		else if(SQ_HFI_POLE_CNT==SQ_HFI_POLE_TIME*4)
//		{
//			SQ_HFI_POLE_INJ_VOLT = 1.0f;
//		}
		else if(SQ_HFI_POLE_CNT==SQ_HFI_POLE_TIME*5)
		{
			SQ_HFI_POLE_INJ_VOLT = 0.0f;	
			SQ_HFI_POLE_CNT=SQ_HFI_POLE_TIME*6+22;
		}
	}
	#endif
	
	if(hfsi.vdh < 0)
	{
		hfsi.dialph_h = -1.0f * hfsi.dialph_h;
		hfsi.dibeta_h = -1.0f * hfsi.dibeta_h;
	}
	else if(hfsi.vdh > 0)
	{
		hfsi.dialph_h = hfsi.dialph_h;
		hfsi.dibeta_h = hfsi.dibeta_h;
	}
	
	#if 1
    // Calculate electrical position using atan2
    hfsi.pos_e = atan2f(hfsi.dibeta_h, hfsi.dialph_h) + M_PI;
    wrap_0_2pi(hfsi.pos_e);
    pll_calc(&hfsi.pll, hfsi.pos_e);
	#endif

	#if 0
		/* Orthogonal PLL */
		ort_pll_calc(&hfsi.pll, hfsi.flux_alpha, hfsi.flux_beta, 1.0f);
		hfsi.pos_e = hfsi.pll.pos_e + SQHFI_POLE_ERR_OFFSTE_ANGLE;
	#endif
	wrap_0_2pi(hfsi.pos_e);

	hfsi.ialph_h_lst = hfsi.ialph_h;
	hfsi.ibeta_h_lst = hfsi.ibeta_h;
	
	hfsi.ialph_lst = pm.foc.i_alph;
	hfsi.ibeta_lst  = pm.foc.i_beta;
	
	hfsi.id_lst = pm.foc.i_d;
	hfsi.iq_lst = pm.foc.i_q;
}



