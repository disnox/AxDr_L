//
// Created by disnox on 2025/6/12.
//
#include "common.h"


void iden_init(void)
{
    memset(&pm.idpm, 0, sizeof(pm.idpm));
    pm.idpm.fs = 20000.0f;
    pm.idpm.ts = 0.00005f;

    // First
    pm.idpm.steps = 4;
    pm.idpm.hstep = pm.idpm.steps / 2;
    pm.idpm.samps = 10;
    pm.idpm.is_max = 5.0f;
    pm.idpm.is_thre = 0.1f;
    pm.idpm.vs_step = 0.0001f;

    pm.idpm.L_cycle = 500;
    pm.idpm.Lts = 0.0001f;
    pm.idpm.vd_plus = +0.6f;
    pm.idpm.vd_minu = -0.6f;
    pm.idpm.vq_plus = +0.6f;
    pm.idpm.vq_minu = -0.6f;

    pm.idpm.fiq_ref = 5.0f;
    pm.idpm.we_l = 500.0f;//1000.0f;   // 低速参考值
    pm.idpm.we_h = 1000.0f; // 高速参考值
    pm.idpm.we_acc = 500.0f;    // 加速度设定

    pm.idpm.Jiq_max = 1.0f;
    pm.idpm.Jfs = 1.0f; // Hz
    pm.idpm.Jts = 1.0f/pm.idpm.Jfs;
    pm.idpm.Js_samps = 6.0f;
    pm.idpm.lambda = 31.4f;
    pm.idpm.Js_init= 1e-3f;
	
	pm.idpm.id_state    = id_Rs;
    pm.idpm.id_Rs_state = id_Rs_init;
	pm.idpm.id_Ls_state = id_Ls_init;
    pm.idpm.id_Fs_state = id_Fs_init;
    pm.idpm.id_Js_state = id_Js_init;


//    pm.idpm.id_state    = id_RL;
//    pm.idpm.id_RL_state = id_RL_init;
//    pm.idpm.id_Fx_state = id_Fx_init;
//    pm.idpm.id_JB_state = id_JB_init;
}

_RAM_FUNC void iden_pmsm_first(idpm_t *x)
{
    static uint16_t wait_cnt = 0;

    x->vs     = pm.foc.vs;
    x->v_alph = pm.foc.v_alph;
    x->v_beta = pm.foc.v_beta;
    x->i_alph = pm.foc.i_alph;
    x->i_beta = pm.foc.i_beta;
    x->v_d    = pm.foc.v_d;
    x->v_q    = pm.foc.v_q;
    x->i_d    = pm.foc.i_d;
    x->i_q    = pm.foc.i_q;
    x->iq_f   = pm.foc.iq_f;
    x->wr     = pm.foc.wr_f;
    x->we     = x->wr * pm.para.pn; // 电角速度

    switch (x->id_state) {
        case id_Rs:
            foc_volt(&pm, x->vd_ref, x->vq_ref, 0.0f);
            iden_Rs(x);
            if (x->id_Rs_state == id_Rs_end) {
                pm.para.Rs = x->Rs;
                x->id_state = id_Ls;
                x->id_Ls_state = id_Ls_init;
            }
            break;

        case id_Ls:
            foc_volt(&pm, x->vd_ref, x->vq_ref, 0.0f);
            iden_Ls(x);
            if (x->id_Ls_state == id_Ls_end) {
                pm.para.Ls = x->Ls;
                pm.para.Ld = x->Ld;
                pm.para.Lq = x->Lq;
                pm.para.Ldif = x->Ldif;
                foc_cur_pi_calc(&pm);
                x->id_state = id_Fs;
                x->id_Fs_state = id_Fs_init;
            }
            break;

        case id_Fs:
            foc_curr(&pm, 0.0f, x->fiq_ref, x->p_e);
            iden_Fs(x);
            if (x->id_Fs_state == id_Fs_end){
                pm.para.flux = x->flux;
                x->id_state = id_Js;
                x->id_Js_state = id_Js_init;
            }
            break;

        case id_Js:
            foc_curr(&pm, 0.0f, x->Jiq_ref, pm.foc.p_e);
            iden_Js(x);
            if (x->id_Js_state == id_Js_end) {
                if (++wait_cnt == 10000) { // 50us * 10000
                    wait_cnt = 0;
                    pm.para.B = x->B;
                    pm.para.Js = x->Js;
                    foc_spd_pi_calc(&pm);
                    x->id_state = id_end;
                }
            }
            break;

        case id_end:
            // ✅ 复位所有子状态机
            x->id_state     = id_Rs;
            x->id_Rs_state  = id_Rs_init;
            x->id_Ls_state  = id_Ls_init;
            x->id_Fs_state  = id_Fs_init;
            x->id_Js_state  = id_Js_init;
		
			pm.ctrl_bit = reset;
			pm.mode.sys = release_mode;
			pm.flag.bit.idpm_done = 1;
            break;

        default:
            break;
    }
}
_RAM_FUNC void iden_Rs(idpm_t *x)
{
    static uint8_t cnt  = 0;
    static uint8_t step = 0;
    static float vs[10];
    static float is[10];
    static float vs_avg = 0.0f;
    static float is_avg = 0.0f;
    static float vs_ref = 0.0f;
    static float is_ref[10];
	
    switch (x->id_Rs_state)
    {
    case id_Rs_init:
    {
        cnt = 0;
        step = 0;
        vs_ref = 0.0f;
        vs_avg = 0.0f;
        is_avg = 0.0f;
        float step_size = x->is_max / ((float)x->hstep);

        for (int i = 0; i < x->hstep; i++)
        {
            is_ref[i] = -step_size * (i + 1);
        }

        for (int i = 0; i < x->hstep; i++)
        {
            is_ref[x->hstep + i] = -is_ref[i];
        }

        x->id_Rs_state = id_Rs_volt;
        break;
    }

    case id_Rs_volt:
    {
        x->i_s = sqrtf(SQ(x->i_d) + SQ(x->i_q));
        float is_err = fabsf(is_ref[step]) - x->i_s;

        if (is_ref[step] >= 0.0f)
        {
            if (is_err > x->is_thre)
            {
                vs_ref += x->vs_step;
            }
            else if (is_err < -x->is_thre)
            {
                vs_ref -= x->vs_step;
            }
        }
        else
        {
            if (is_err > x->is_thre)
            {
                vs_ref -= x->vs_step;
            }
            else if (is_err < -x->is_thre)
            {
                vs_ref += x->vs_step;
            }
        }

        x->vd_ref = vs_ref;

        if (fabs(is_err) < x->is_thre)
        {
            cnt = 0;
            vs_avg = 0.0f;
            is_avg = 0.0f;
            x->id_Rs_state = id_Rs_wait; // 进入等待稳定阶段
        }
    }
    break;

    case id_Rs_wait: // 等待采样稳定阶段
    {
        if (++cnt >= 100)   // 等待大约 5ms (100 * 50us)
        {
            x->id_Rs_state = id_Rs_samp;  // 进入采样阶段
            cnt = 0;  // 重置采样延时计数器
        }
    }
    break;

    case id_Rs_samp: // 采样阶段
    {
        vs_avg += sqrtf(SQ(x->v_d) + SQ(x->v_q));
        is_avg += sqrtf(SQ(x->i_d) + SQ(x->i_q));
        cnt++;

        if (cnt >= x->samps)
        {
            vs[step] = vs_avg / x->samps;
            is[step] = is_avg / x->samps;
            step++;

            if (step < x->steps)
            {
                vs_ref = 0.0f; // 重置电压调整
                x->id_Rs_state = id_Rs_volt;  // 进入下一个电流阶梯
            }
            else
            {
				x->id_Rs_state = id_Rs_calc;  // 采样结束，进入计算阶段
                step = 0;
                vs_ref = 0.0f;
                x->vd_ref = 0.0f;  
            }
        }
    }
    break;

    case id_Rs_calc:   // 计算电阻阶段
    {
        float res_sum = 0.0f;

        // 计算正阶梯的电阻
        for (int i = 0; i < x->hstep - 1; i++)
        {
            float delta_volt = vs[i + 1] - vs[i];
            float delta_curr = is[i + 1] - is[i];
            res_sum += delta_volt / delta_curr;
        }

        // 计算负阶梯的电阻
        for (int i = x->hstep; i < x->steps - 1; i++)
        {
            float delta_volt = vs[i + 1] - vs[i];
            float delta_curr = is[i + 1] - is[i];
            res_sum += delta_volt / delta_curr;
        }

        // 取两部分的平均值
        x->Rs = res_sum / (x->steps - 2);
        x->id_Rs_state = id_Rs_end;
        break;
    }

    case id_Rs_end: // 结束状态
        break;
	default:
        break;
    }
	
}

float ids[2] = {0.0f, 0.0f};
float iqs[2] = {0.0f, 0.0f};
_RAM_FUNC void iden_Ls(idpm_t *x)
{
    // 定义变量
    static uint8_t icnt = 0;
    static uint32_t loop_count = 0;
	
    switch (x->id_Ls_state)
    {
    case id_Ls_init:
    {
        loop_count = 0;
        ids[0]  = 0.0f;
        ids[1]  = 0.0f;
        iqs[0]  = 0.0f;
        iqs[1]  = 0.0f;
        x->vd_ref = 0.0f;
        x->vq_ref = 0.0f;
        x->id_Ls_state = id_Ld_loop;
    }
    break;

    case id_Ld_loop:
    {
        if (++x->Lcnt == 10)
        {
            x->Lcnt = 0;

            if (loop_count < x->L_cycle)
            {
                x->vd_ref = (icnt == 0) ? x->vd_minu : x->vd_plus;
            }

            ids[icnt] += x->i_d;
            icnt = loop_count & 1;

            if (loop_count >= x->L_cycle)
            {
                x->Lcnt = 0;
                x->vd_ref = 0.0f;
                loop_count = 0;
                x->id_Ls_state = id_Lq_loop;
            }
            else
            {
                x->id_Ls_state = id_Ld_loop;
            }

            loop_count++;
        }
    }
    break;

    case id_Lq_loop:
    {
        if (++x->Lcnt == 10)
        {
            x->Lcnt = 0;

            if (loop_count < x->L_cycle)
            {
                x->vq_ref = (icnt == 0) ? x->vq_minu : x->vq_plus;
            }

            iqs[icnt] += x->i_q; // id 电流反馈
            icnt = loop_count & 1;

            if (loop_count >= x->L_cycle)
            {
                x->vq_ref = 0.0f;
                loop_count = 0;
                x->id_Ls_state = id_Ls_calc;
            }
            else
            {
                x->id_Ls_state = id_Lq_loop;
            }

            loop_count++;
        }
    }
    break;

    case id_Ls_calc:
    {
        // 计算 d 轴电感 Ld+ 和 Ld-
        float dId_by_dt_plus  = ids[0] / (x->ts *x->L_cycle);
        float dId_by_dt_minus = ids[1] / (x->ts *x->L_cycle);
        x->Ld_plus = (x->vd_plus - x->Rs *ids[1] * x->ts) / dId_by_dt_plus;
        x->Ld_minu = (x->vd_minu - x->Rs *ids[0] * x->ts) / dId_by_dt_minus;
        // 计算 q 轴电感 Lq+ 和 Lq-
        float dIq_by_dt_plus  = iqs[0] / (x->ts *x->L_cycle);
        float dIq_by_dt_minus = iqs[1] / (x->ts *x->L_cycle);
        x->Lq_plus = (x->vq_plus - x->Rs *iqs[1] * x->ts) / dIq_by_dt_plus;
        x->Lq_minu = (x->vq_minu - x->Rs *iqs[0] * x->ts) / dIq_by_dt_minus;
        x->Ld = (x->Ld_plus + x->Ld_minu) * 0.5f;
        x->Lq = (x->Lq_plus + x->Lq_minu) * 0.5f;
		x->Ldif = fabsf(x->Ld - x->Lq);
        x->Ls = (x->Ld + x->Lq) * 0.5f;
        x->id_Ls_state = id_Ls_end;
    }
    break;
	
	case id_Ls_end:
		break;

    default:
        break;
    }
}

_RAM_FUNC void iden_Fs(idpm_t *x)
{
	static float vq_avg_l, vd_avg_l, id_avg_l, iq_avg_l;
	static float vq_avg_h, vd_avg_h, id_avg_h, iq_avg_h;
	static float sample_l, sample_h;
	static uint32_t wait_cnt = 0; // 计数器
	
    x->p_e += x->pe_acc;
    wrap_pm_pi(x->p_e);
	
    switch (x->id_Fs_state)
    {
    case id_Fs_init:
	{
		x->vs_l = 0.0f;
		x->vs_h = 0.0f;
		x->is_l = 0.0f;
		x->is_h = 0.0f;
        x->id_Fs_state = id_Fs_spd_l;
	}
    break;

    case id_Fs_spd_l:
    {
        x->we_ref = x->we_l;

		if (x->we_ref > x->we_set)
		{
			x->we_set += x->we_acc * x->ts;
			if (x->we_set > x->we_ref)
				x->we_set = x->we_ref;
		}
		else if (x->we_ref < x->we_set)
		{
			x->we_set -= x->we_acc * x->ts;
			if (x->we_set < x->we_ref)
				x->we_set = x->we_ref;
		}

		x->pe_acc = x->we_set * x->ts;
		
        if (x->we_set == x->we_ref)
        {
            // 等待速度到达
            if (++wait_cnt >= 10000)
            {
                vd_avg_l = 0.0f;
                vq_avg_l = 0.0f;
                id_avg_l = 0.0f;
                iq_avg_l = 0.0f;
                sample_l = 0.0f;
                wait_cnt = 0;                 // 重置等待计数器
                x->id_Fs_state = id_Fs_samp_l;  // 进入等待状态
            }
        }
    }
    break;

    case id_Fs_samp_l:
    {
        vd_avg_l += x->v_d;
        vq_avg_l += x->v_q;
        id_avg_l += x->i_d;
        iq_avg_l += x->i_q;
        sample_l += 1.0f;

        if (++wait_cnt >= 20000)
        {
            wait_cnt = 0;
            vd_avg_l /= sample_l;
            vq_avg_l /= sample_l;
            id_avg_l /= sample_l;
            iq_avg_l /= sample_l;
            x->vs_l = sqrtf(SQ(vd_avg_l) + SQ(vq_avg_l));
            x->is_l = sqrtf(SQ(id_avg_l) + SQ(iq_avg_l));
            x->id_Fs_state = id_Fs_spd_h;  // 进入高速状态
        }
    }
    break;

    case id_Fs_spd_h:
    {
        x->we_ref = x->we_h;
        if (x->we_ref > x->we_set)
		{
			x->we_set += x->we_acc * x->ts;
			if (x->we_set > x->we_ref)
				x->we_set = x->we_ref;
		}
		else if (x->we_ref < x->we_set)
		{
			x->we_set -= x->we_acc * x->ts;
			if (x->we_set < x->we_ref)
				x->we_set = x->we_ref;
		}

		x->pe_acc = x->we_set * x->ts;
		
        if (x->we_set == x->we_ref)
        {
            if (++wait_cnt >= 10000)
            {
                vd_avg_h = 0.0f;
                vq_avg_h = 0.0f;
                id_avg_h = 0.0f;
                iq_avg_h = 0.0f;
                sample_h = 0.0f;
                wait_cnt = 0;                 
                x->id_Fs_state = id_Fs_samp_h;
            }
        }
    }
    break;

    case id_Fs_samp_h:
    {
        vd_avg_h += x->v_d;
        vq_avg_h += x->v_q;
        id_avg_h += x->i_d;
        iq_avg_h += x->i_q;
        sample_h += 1.0f;

        if (++wait_cnt >= 20000)
        {
            wait_cnt = 0;
            vd_avg_h /= sample_h;
            vq_avg_h /= sample_h;
            id_avg_h /= sample_h;
            iq_avg_h /= sample_h;
            x->vs_h = sqrtf(SQ(vd_avg_h) + SQ(vq_avg_h));
            x->is_h = sqrtf(SQ(id_avg_h) + SQ(iq_avg_h));
			
			x->pe_acc = 0;
			
            x->id_Fs_state = id_Fs_calc;
        }
    }
    break;

    case id_Fs_calc:
    {
        // 计算磁链，高速和低速电压电流差值来求磁链
        float del_vs = x->vs_h - x->vs_l;  // 电压差
        float del_is = x->is_h - x->is_l;  // 电流差
        float del_we = x->we_h - x->we_l;  // 速度差
        // 根据高速-低速区间的电压和电流差值计算磁链
        x->flux = (del_vs - x->Rs * del_is) / del_we - del_is * x->Ls;
		x->id_Fs_state = id_Fs_end;
    }
    break;
	case id_Fs_end:
		break;
    default:
        break;
    }
}

_RAM_FUNC void iden_Js(idpm_t *x)
{
	static uint8_t wait_cnt = 0;
	static uint8_t samp_cnt = 0;
	static float tc = 0.0;
	static float q0 = 0.0;
	static float q1 = 0.0;
	static float q1_dot = 0.0;
	static float Tem_est = 0.0;
	static float Js_avg = 0.0f;
	static float Js_est = 0.0f;
	static float Js_fac = 0.0f;
    static float sumx = 0.0;
	static float sumy = 0.0;
	
	x->Jiq_ref = x->Jiq_max*sin_f32(M_2PI*x->Jfs*x->Jtc);
	
	switch (x->id_Js_state)
    {
		case id_Js_init:
		{
			x->id_Js_state = id_Js_loop;
		}
		break;
		case id_Js_loop:
		{
		    x->tc += x->ts;
			x->Jtc += x->ts;
			
			x->Tem = 1.5f*pm.para.pn*x->flux*x->i_q; 
			
			q0 += x->lambda*(-q0 + x->Tem) * x->ts;
			q1_dot = x->lambda*(-q1 + x->wr);
			q1 += q1_dot * x->ts;
			Tem_est = q0 - (x->Js_init) * q1_dot;
			sumx += (Tem_est *q1_dot)*x->ts;
			sumy += (q1_dot *q1_dot)*x->ts;

			if (x->tc >= x->Jts)
			{
				if (sumy < 0.001f)
					sumy = 0.001f; // avoid zero denominator

				Js_fac = + sumx / sumy;
				Js_est = x->Js_init + Js_fac;
				x->tc = 0.0;
				sumx = 0.0;
				sumy = 0.0;
				
				if(++wait_cnt >= 2)
				{
					Js_avg += Js_est;
					if(++samp_cnt >= x->Js_samps)
						x->id_Js_state = id_Js_calc;
				}
				
			}
		}
		break;
		case id_Js_calc:
		{
			x->Jtc = 0.0f;
			x->Jiq_ref = 0.0f;
			x->Js = Js_avg/x->Js_samps;
			x->id_Js_state = id_Js_end;
		}
		break;
		
		case id_Js_end:
			break;
		default:
			break;
	}
}

