#include "common.h"

/**
***********************************************************************
* @brief:      foc_calc(pmsm_foc_t *foc)
* @param[in]:  foc  Pointer to FOC parameter structure
* @retval:     void
* @details:    FOC algorithm: coordinate transformation and voltage calculation
***********************************************************************
**/
_RAM_FUNC void foc_calc(pmsm_foc_t *foc)
{
    foc->sin_val = sinf(foc->theta);
    foc->cos_val = cosf(foc->theta);
    foc->i_alph = foc->i_a;
    foc->i_beta = (foc->i_b - foc->i_c) * ONE_BY_SQRT3;
    foc->i_d = foc->i_alph * foc->cos_val + foc->i_beta * foc->sin_val;
    foc->i_q = foc->i_beta * foc->cos_val - foc->i_alph * foc->sin_val;
    foc->v_alph = (foc->v_d *foc->cos_val - foc->v_q *foc->sin_val);
    foc->v_beta = (foc->v_d *foc->sin_val + foc->v_q *foc->cos_val);
    svm(foc->v_alph*(foc->inv_vbus), foc->v_beta*(foc->inv_vbus), &foc->dtc_a, &foc->dtc_b, &foc->dtc_c);
}

/**
***********************************************************************
* @brief:      sin_cos_val(pmsm_foc_t *foc)
* @param[in]:  foc  FOC参数结构体指针
* @retval:     void
* @details:    计算角度 theta 对应的 sin 和 cos 值
***********************************************************************
**/
_RAM_FUNC void sin_cos_val(pmsm_foc_t *foc)
{
    foc->sin_val = sinf(foc->theta);
    foc->cos_val = cosf(foc->theta);
}

/**
***********************************************************************
* @brief:      clarke_transform(pmsm_foc_t *foc)
* @param[in]:  foc  FOC参数结构体指针
* @retval:     void
* @details:    Clarke变换，将三相电流变换为Alpha-Beta坐标系下的电流
***********************************************************************
**/
_RAM_FUNC void clarke_transform(pmsm_foc_t *foc)
{
    foc->i_alph = foc->i_a;
    foc->i_beta = (foc->i_b - foc->i_c) * ONE_BY_SQRT3;
}

/**
***********************************************************************
* @brief:      inverse_clarke(pmsm_foc_t *foc)
* @param[in]:  foc  FOC参数结构体指针
* @retval:     void
* @details:    逆Clarke变换，将 Alpha-Beta 坐标系下的电压转换为三相电压
***********************************************************************
**/
_RAM_FUNC void inverse_clarke(pmsm_foc_t *foc)
{
    foc->v_a = foc->v_alph;
    foc->v_b = -0.5f * foc->v_alph + SQRT3_BY_2 *foc->v_beta;
    foc->v_c = -0.5f * foc->v_alph - SQRT3_BY_2 *foc->v_beta;
}

/**
***********************************************************************
* @brief:      park_transform(pmsm_foc_t *foc)
* @param[in]:  foc  FOC参数结构体指针
* @retval:     void
* @details:    Park变换，将 Alpha-Beta 坐标系下的电流转换为 dq 坐标系下的电流
***********************************************************************
**/
_RAM_FUNC void park_transform(pmsm_foc_t *foc)
{
    foc->i_d = foc->i_alph * foc->cos_val + foc->i_beta * foc->sin_val;
    foc->i_q = foc->i_beta * foc->cos_val - foc->i_alph * foc->sin_val;
}

/**
***********************************************************************
* @brief:      inverse_park(pmsm_foc_t *foc)
* @param[in]:  foc  FOC参数结构体指针
* @retval:     void
* @details:    逆Park变换，将 dq 坐标系下的电压转换为 Alpha-Beta 坐标系下的电压
***********************************************************************
**/
_RAM_FUNC void inverse_park(pmsm_foc_t *foc)
{
    foc->v_alph = (foc->v_d *foc->cos_val - foc->v_q *foc->sin_val);
    foc->v_beta = (foc->v_d *foc->sin_val + foc->v_q *foc->cos_val);
}

_RAM_FUNC void svpwm_midpoint(pmsm_foc_t *foc)
{
    foc->v_alph = foc->inv_vbus *foc->v_alph;
    foc->v_beta = foc->inv_vbus *foc->v_beta;
    float va = foc->v_alph;
    float vb = -0.5f * foc->v_alph + SQRT3_BY_2 *foc->v_beta;
    float vc = -0.5f * foc->v_alph - SQRT3_BY_2 *foc->v_beta;
    float vmax = max(max(va, vb), vc);
    float vmin = min(min(va, vb), vc);
    float vcom = (vmax + vmin) * 0.5f;
    foc->dtc_a = 1.0f-((va - vcom) + 0.5f);
    foc->dtc_b = 1.0f-((vb - vcom) + 0.5f);
    foc->dtc_c = 1.0f-((vc - vcom) + 0.5f);
}

_RAM_FUNC void svpwm_sector(pmsm_foc_t *foc)
{
    float TS = 1.0f;
    float ta = 0.0f, tb = 0.0f, tc = 0.0f;
    float k = (TS *SQRT3) * foc->inv_vbus;
    float va = foc->v_beta;
    float vb = (SQRT3 *foc->v_alph - foc->v_beta) * 0.5f;
    float vc = (-SQRT3 *foc->v_alph - foc->v_beta) * 0.5f;
    int a = (va > 0.0f) ? 1 : 0;
    int b = (vb > 0.0f) ? 1 : 0;
    int c = (vc > 0.0f) ? 1 : 0;
    int sextant = (c << 2) + (b << 1) + a;

    switch (sextant)
    {
    case 3:
    {
        float t4 = k *vb;
        float t6 = k *va;
        float t0 = (TS - t4 - t6) * 0.5f;
        ta = t4 + t6 + t0;
        tb = t6 + t0;
        tc = t0;
    }
    break;

    case 1:
    {
        float t6 = -k *vc;
        float t2 = -k *vb;
        float t0 = (TS - t2 - t6) * 0.5f;
        ta = t6 + t0;
        tb = t2 + t6 + t0;
        tc = t0;
    }
    break;

    case 5:
    {
        float t2 = k *va;
        float t3 = k *vc;
        float t0 = (TS - t2 - t3) * 0.5f;
        ta = t0;
        tb = t2 + t3 + t0;
        tc = t3 + t0;
    }
    break;

    case 4:
    {
        float t1 = -k *va;
        float t3 = -k *vb;
        float t0 = (TS - t1 - t3) * 0.5f;
        ta = t0;
        tb = t3 + t0;
        tc = t1 + t3 + t0;
    }
    break;

    case 6:
    {
        float t1 = k *vc;
        float t5 = k *vb;
        float t0 = (TS - t1 - t5) * 0.5f;
        ta = t5 + t0;
        tb = t0;
        tc = t1 + t5 + t0;
    }
    break;

    case 2:
    {
        float t4 = -k *vc;
        float t5 = -k *va;
        float t0 = (TS - t4 - t5) * 0.5f;
        ta = t4 + t5 + t0;
        tb = t0;
        tc = t5 + t0;
    }
    break;

    default:
        break;
    }

    foc->dtc_a = 1.0f - ta;
    foc->dtc_b = 1.0f - tb;
    foc->dtc_c = 1.0f - tc;
}

int svm(float alpha, float beta, float *ta, float *tb, float *tc)
{
    int Sextant;
	float tA, tB, tC;
    if (beta >= 0.0f)
    {
        if (alpha >= 0.0f)
        {
            //quadrant I
            if (ONE_BY_SQRT3 *beta > alpha)
            {
                Sextant = 2;    //sextant v2-v3
            }
            else
            {
                Sextant = 1;    //sextant v1-v2
            }
        }
        else
        {
            //quadrant II
            if (-ONE_BY_SQRT3 *beta > alpha)
            {
                Sextant = 3;    //sextant v3-v4
            }
            else
            {
                Sextant = 2;    //sextant v2-v3
            }
        }
    }
    else
    {
        if (alpha >= 0.0f)
        {
            //quadrant IV
            if (-ONE_BY_SQRT3 *beta > alpha)
            {
                Sextant = 5;    //sextant v5-v6
            }
            else
            {
                Sextant = 6;    //sextant v6-v1
            }
        }
        else
        {
            //quadrant III
            if (ONE_BY_SQRT3 *beta > alpha)
            {
                Sextant = 4;    //sextant v4-v5
            }
            else
            {
                Sextant = 5;    //sextant v5-v6
            }
        }
    }

    switch (Sextant)
    {
    // sextant v1-v2
    case 1:
    {
        // Vector on-times
        float t1 = alpha - ONE_BY_SQRT3 *beta;
        float t2 = TWO_BY_SQRT3 *beta;
        // PWM timings
        tA = (1.0f - t1 - t2) * 0.5f;
        tB = tA + t1;
        tC = tB + t2;
    }
    break;

    // sextant v2-v3
    case 2:
    {
        // Vector on-times
        float t2 = alpha + ONE_BY_SQRT3 *beta;
        float t3 = -alpha + ONE_BY_SQRT3 *beta;
        // PWM timings
        tB = (1.0f - t2 - t3) * 0.5f;
        tA = tB + t3;
        tC = tA + t2;
    }
    break;

    // sextant v3-v4
    case 3:
    {
        // Vector on-times
        float t3 = TWO_BY_SQRT3 *beta;
        float t4 = -alpha - ONE_BY_SQRT3 *beta;
        // PWM timings
        tB = (1.0f - t3 - t4) * 0.5f;
        tC = tB + t3;
        tA = tC + t4;
    }
    break;

    // sextant v4-v5
    case 4:
    {
        // Vector on-times
        float t4 = -alpha + ONE_BY_SQRT3 *beta;
        float t5 = -TWO_BY_SQRT3 *beta;
        // PWM timings
        tC = (1.0f - t4 - t5) * 0.5f;
        tB = tC + t5;
        tA = tB + t4;
    }
    break;

    // sextant v5-v6
    case 5:
    {
        // Vector on-times
        float t5 = -alpha - ONE_BY_SQRT3 *beta;
        float t6 = alpha - ONE_BY_SQRT3 *beta;
        // PWM timings
        tC = (1.0f - t5 - t6) * 0.5f;
        tA = tC + t5;
        tB = tA + t6;
    }
    break;

    // sextant v6-v1
    case 6:
    {
        // Vector on-times
        float t6 = -TWO_BY_SQRT3 *beta;
        float t1 = alpha + ONE_BY_SQRT3 *beta;
        // PWM timings
        tA = (1.0f - t6 - t1) * 0.5f;
        tC = tA + t1;
        tB = tC + t6;
    }
    break;
    }
	
    *ta = tA;
    *tb = tB;
    *tc = tC;

    int result_valid = *ta >= 0.0f && *ta <= 1.0f && *tb >= 0.0f && *tb <= 1.0f && *tc >= 0.0f && *tc <= 1.0f;
    // if any of the results becomes NaN, result_valid will evaluate to false
   
    return result_valid ? 0 : -1;
}
