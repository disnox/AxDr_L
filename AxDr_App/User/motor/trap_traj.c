//
// Created by disno on 2025/7/10.
//
#include "common.h"
#include <math.h>

float A1,B1,C1,F1;
float A2,B2,C2,F2;

_RAM_FUNC void traj_init(void)
{
    memset(&pm.traj, 0, sizeof(traj_t));

    pm.traj.ts = pm.period.foc_ts; // 20kHz
}

_RAM_FUNC float sign_hard(float val)
{
    return (signbit(val)) ? -1.0f : 1.0f;
}

_RAM_FUNC void spd_traj_plan(traj_t *x, float vel_init, float vel_end, float acc, float dec, float curve_mode)
{
    x->curve_mode = curve_mode;
    x->vel_init   = vel_init;
    x->vel 		  = vel_end;

    if(vel_init>=0 && vel_end>=0 && vel_init<vel_end)  // 正向加速
    {
        x->flag = 1;
        x->acc = acc;
    }
    if(vel_init>=0 && vel_end>=0 && vel_init>vel_end) // 正向减速
    {
        x->flag = 2;
        x->dec = -dec;
    }

    if(vel_init<=0 && vel_end<=0 && vel_init>vel_end) // 反向加速
    {
        x->flag = 3;
        x->acc = -acc;
    }
    if(vel_init<=0 && vel_end<=0 && vel_init<vel_end) // 反向减速
    {
        x->flag = 4;
        x->dec = dec;
    }
    if(vel_init<0 && vel_end>0 && vel_init<vel_end) // 反向减速 正向加速
    {
        x->flag = 5;
        x->acc = acc;
        x->dec = dec;
    }
    if(vel_init>0 && vel_end<0 && vel_init>vel_end) // 正向减速 反向加速
    {
        x->flag = 6;
        x->acc = -acc;
        x->dec = -dec;
    }

    switch (x->curve_mode) 
    {
        case tcurve:
            // Calculate acceleration and deceleration times
            if (x->flag==1||x->flag==3) {
                x->t_acc = (vel_end - vel_init) / x->acc;
                x->t_dec = 0;	
            }
            if (x->flag==2||x->flag==4) {
                x->t_acc = 0;
                x->t_dec = (vel_end - vel_init) / x->dec;
            }
            if (x->flag==5||x->flag==6) {
                x->t_acc = (vel_end - 0) / x->acc;
                x->t_dec = (0 - vel_init) / x->dec;
            }
        break;

        case scurve:
            // Calculate acceleration and deceleration times
            if (x->flag==1||x->flag==3) {
                x->t_acc = (vel_end - vel_init) / x->acc;
                x->t_dec = 0;	
                A1 = 6.0f * (x->vel - x->vel_init) / 6.0f;
                B1 = 15.0f * (x->vel_init - x->vel) / 5.0f;
                C1 = 10.0f * (x->vel - x->vel_init) / 4.0f;
                F1 = x->vel_init;		
            }
            if (x->flag==2||x->flag==4) {
                x->t_acc = 0;
                x->t_dec = (vel_end - vel_init) / x->dec;
                A1 = 6.0f * (x->vel - x->vel_init) / 6.0f;
                B1 = 15.0f * (x->vel_init - x->vel) / 5.0f;
                C1 = 10.0f * (x->vel - x->vel_init) / 4.0f;
                F1 = x->vel_init;
            }
            if (x->flag==5||x->flag==6) {
                x->t_acc = (vel_end - 0) / x->acc;
                x->t_dec = (0 - vel_init) / x->dec;
                A1 = 6.0f * (0 - x->vel_init) / 6.0f;
                B1 = 15.0f * (x->vel_init - 0) / 5.0f;
                C1 = 10.0f * (0 - x->vel_init) / 4.0f;
                F1 = x->vel_init;

                A2 = 6.0f * (x->vel - 0) / 6.0f;
                B2 = 15.0f * (0 - x->vel) / 5.0f;
                C2 = 10.0f * (x->vel - 0) / 4.0f;
                F2 = 0;
            }
        break;
    }

    // For speed planning, we only consider acceleration and then maintaining target speed
    x->t_total = x->t_acc + x->t_dec;

    x->tick = 0;
    x->profile_done = false;
    x->spd_step = vel_init;
}


_RAM_FUNC void spd_traj_eval(traj_t *x)
{
    if (x->profile_done) {
        return;
    }

    x->tick++;
    float t = x->tick * x->ts;

    switch (x->curve_mode) {
        case tcurve:
            if (x->flag <= 4) {
                if (t < x->t_acc) { // Accelerating
                    x->spd_step = x->vel_init + x->acc * t;
                } else if (t < x->t_dec) { // Decelerating
                    x->spd_step = x->vel_init + x->dec * t;
                } else { // Final Condition
                    x->spd_step = x->vel;
                    x->profile_done = true;
                }
            } else if (x->flag == 5 || x->flag == 6) {
                if (t < x->t_dec) { // Decelerating
                    x->spd_step = x->vel_init + x->dec * t;
                } else if (t < x->t_total) { // Accelerating
                    float t_acc = t - x->t_dec;
                    x->spd_step = 0 + x->acc * t_acc;
                } else { // Final Condition
                    x->spd_step = x->vel;
                    x->profile_done = true;
                }
            }
            break;
        case scurve:
            if (t < 0.0f) { // Initial Condition
                x->spd_step = x->vel_init;
                break;
            }

            if (x->flag <= 4) {
                if (t < x->t_acc) { // Accelerating
                    float t_ratio = t / x->t_acc;
                    float t_ratio_pow_2 = t_ratio * t_ratio;
                    float t_ratio_pow_3 = t_ratio_pow_2 * t_ratio;
                    float t_ratio_pow_4 = t_ratio_pow_3 * t_ratio;
                    float t_ratio_pow_5 = t_ratio_pow_4 * t_ratio;

                    x->spd_step = (6.0f * A1 * t_ratio_pow_5 + 5.0f * B1 * t_ratio_pow_4 + 4.0f * C1 * t_ratio_pow_3 + F1);
                } else if (t < x->t_dec) { // Decelerating
                    float t_ratio = t / x->t_dec;
                    float t_ratio_pow_2 = t_ratio * t_ratio;
                    float t_ratio_pow_3 = t_ratio_pow_2 * t_ratio;
                    float t_ratio_pow_4 = t_ratio_pow_3 * t_ratio;
                    float t_ratio_pow_5 = t_ratio_pow_4 * t_ratio;

                    x->spd_step = (6.0f * A1 * t_ratio_pow_5 + 5.0f * B1 * t_ratio_pow_4 + 4.0f * C1 * t_ratio_pow_3 + F1);
                } else { // Final Condition
                    x->spd_step = x->vel;
                    x->profile_done = true;
                }
            } else if (x->flag == 5 || x->flag == 6) {
                if (t < x->t_dec) { // Decelerating
                    float t_ratio = t / x->t_dec;
                    float t_ratio_pow_2 = t_ratio * t_ratio;
                    float t_ratio_pow_3 = t_ratio_pow_2 * t_ratio;
                    float t_ratio_pow_4 = t_ratio_pow_3 * t_ratio;
                    float t_ratio_pow_5 = t_ratio_pow_4 * t_ratio;

                    x->spd_step = (6.0f * A1 * t_ratio_pow_5 + 5.0f * B1 * t_ratio_pow_4 + 4.0f * C1 * t_ratio_pow_3 + F1);
                } else if (t < x->t_total) { // Accelerating
                    float t_acc = t - x->t_dec;
                    float t_ratio = t_acc / x->t_acc;
                    float t_ratio_pow_2 = t_ratio * t_ratio;
                    float t_ratio_pow_3 = t_ratio_pow_2 * t_ratio;
                    float t_ratio_pow_4 = t_ratio_pow_3 * t_ratio;
                    float t_ratio_pow_5 = t_ratio_pow_4 * t_ratio;

                    x->spd_step = (6.0f * A2 * t_ratio_pow_5 + 5.0f * B2 * t_ratio_pow_4 + 4.0f * C2 * t_ratio_pow_3 + F2);
                } else { // Final Condition
                    x->spd_step = x->vel;
                    x->profile_done = true;
                }
            }
            break;
    }
}


_RAM_FUNC void pos_traj_plan(traj_t *x, float pos_init, float pos_end, float vel_init, float v_max, float acc, float dec, float curve_mode)
{
    float distance  = pos_end - pos_init;           // Distance to travel
    float stop_dist = SQ(vel_init) / (2.0f * dec);  // Minimum stopping distance
    float dXstop    = copysign(stop_dist, vel_init); // Minimum stopping displacement
    float s         = sign_hard(distance - dXstop);        // Sign of coast velocity (if any)
    v_max         = ABS(v_max);                         
    x->acc        = s * acc;                            // Maximum Acceleration (signed)
    x->dec        = -s * dec;                           // Maximum Deceleration (signed)
    x->vel        = s * v_max;                            // Maximum Velocity (signed)

    x->curve_mode = curve_mode;

    // If we start with a speed faster than cruising, then we need to decel instead of accel aka "double deceleration move" in the paper
    if ((s * vel_init) > (s * x->vel)) {
        x->acc = -s * acc;
    }

    // Time to accel/decel to/from Vr (cruise speed)
    x->t_acc = (x->vel - vel_init) / x->acc;
    x->t_dec = -x->vel / x->dec;

    // Integral of velocity ramps over the full accel and decel times to get
    // minimum displacement required to reach cuising speed
    float dXmin = 0.5f * x->t_acc * (x->vel + vel_init) + 0.5f * x->t_dec * x->vel;

    // Are we displacing enough to reach cruising speed?
    if (s * distance < s * dXmin) {
        // Short move (triangle profile)
        x->vel = s * sqrtf(fmaxf((x->dec * SQ(vel_init) + 2.0f * x->acc * x->dec * distance) / (x->dec - x->acc), 0.0f));
        x->t_acc = fmaxf(0.0f, (x->vel - vel_init) / x->acc);
        x->t_dec = fmaxf(0.0f, -x->vel / x->dec);
        x->t_vel = 0.0f;
    } else {
        // Long move (trapezoidal profile)
        x->t_vel = (distance - dXmin) / x->vel;
    }

    if (x->curve_mode == scurve) {
        A1 = 1.0f * (x->vel - x->vel_init);
        B1 = 3.0f * (x->vel_init - x->vel);
        C1 = 2.5f * (x->vel - x->vel_init);
        F1 = x->vel_init;

        A2 = 1.0f * (0.0f - x->vel );
        B2 = 3.0f * (x->vel - 0.0f);
        C2 = 2.5f * (0.0f - x->vel);
        F2 = x->vel;
    }

    // Fill in the rest of the values used at evaluation-time
    x->t_total  = x->t_acc + x->t_vel + x->t_dec;
    x->pos_init = pos_init;
    x->vel_init = vel_init;
    x->pos_end  = pos_end;
    x->acc_distance   = pos_init + vel_init * x->t_acc + 0.5f * x->acc * SQ(x->t_acc); // pos at end of accel phase
    x->tick         = 0;
    x->profile_done = false;
}



_RAM_FUNC void pos_traj_eval(traj_t *x) {
    if (x->profile_done) {
        return;
    }

    x->tick++;
    float t = x->tick * x->ts;

    switch (x->curve_mode) {
        case tcurve:
            if (t < 0.0f) { // Initial Condition
                x->pos_step = x->pos_init;
                x->spd_step = x->vel_init;
                x->tor_step = 0.0f;
            } else if (t < x->t_acc) { // Accelerating
                x->pos_step = x->pos_init + x->vel_init * t + 0.5f * x->acc * SQ(t);
                x->spd_step = x->vel_init + x->acc * t;
                x->tor_step = x->acc;
            } else if (t < x->t_acc + x->t_vel) { // Coasting
                x->pos_step = x->acc_distance + x->vel * (t - x->t_acc);
                x->spd_step = x->vel;
                x->tor_step = 0.0f;
            } else if (t < x->t_total) { // Deceleration
                float td = t - (x->t_acc + x->t_vel);
                // 梯形减速
                x->pos_step = x->acc_distance + x->vel * x->t_vel + x->vel * td + 0.5f * x->dec * SQ(td);
                x->spd_step = x->vel + x->dec * td;
                x->tor_step = x->dec;
            } else if (t >= x->t_total) { // Final Condition
                x->pos_step = x->pos_end;
                x->spd_step = 0.0f;
                x->tor_step = 0.0f;
                x->profile_done = true;
            }
            break;
        case scurve:
            if (t < 0.0f) { // Initial Condition
                x->pos_step = x->pos_init;
                x->spd_step = x->vel_init;
                x->tor_step = 0.0f;
            } else if (t < x->t_acc) { // Accelerating
                float t_ratio = t / x->t_acc;
                float t_ratio_pow_2 = t_ratio * t_ratio;
                float t_ratio_pow_3 = t_ratio_pow_2 * t_ratio;
                float t_ratio_pow_4 = t_ratio_pow_3 * t_ratio;
                float t_ratio_pow_5 = t_ratio_pow_4 * t_ratio;
                float t_ratio_pow_6 = t_ratio_pow_5 * t_ratio;

                x->pos_step = x->pos_init + x->t_acc * (A1 * t_ratio_pow_6 + B1 * t_ratio_pow_5 + C1 * t_ratio_pow_4 + F1 * t_ratio);
                x->spd_step = (6.0f * A1 * t_ratio_pow_5 + 5.0f * B1 * t_ratio_pow_4 + 4.0f * C1 * t_ratio_pow_3 + F1);
                x->tor_step = (5.0f * 6.0f * A1 * t_ratio_pow_4 + 4.0f * 5.0f * B1 * t_ratio_pow_3 + 3.0f * 4.0f * C1 * t_ratio_pow_2) / x->t_acc;
            } else if (t < x->t_acc + x->t_vel) { // Coasting
                x->pos_step = x->acc_distance + x->vel * (t - x->t_acc);
                x->spd_step = x->vel;
                x->tor_step = 0.0f;
            } else if (t < x->t_total) { // Deceleration
                float Tb = x->t_acc + x->t_vel;
                float Tc = x->t_total;
                float y = (t - Tb) / (Tc - Tb);
                float x_pow_2 = y * y;
                float x_pow_3 = x_pow_2 * y;
                float x_pow_4 = x_pow_3 * y;
                float x_pow_5 = x_pow_4 * y;
                float x_pow_6 = x_pow_5 * y;

                x->pos_step = x->acc_distance + x->vel * x->t_vel + (Tc - Tb) * (A2 * x_pow_6 + B2 * x_pow_5 + C2 * x_pow_4 + F2 * y);
                x->spd_step = (6.0f * A2 * x_pow_5 + 5.0f * B2 * x_pow_4 + 4.0f * C2 * x_pow_3 + F2);
                x->tor_step = (5.0f * 6.0f * A2 * x_pow_4 + 4.0f * 5.0f * B2 * x_pow_3 + 3.0f * 4.0f * C2 * x_pow_2) / (Tc - Tb);
            } else { // Final Condition
                x->pos_step     = x->pos_end;
                x->spd_step     = 0.0f;
                x->tor_step     = 0.0f;
                x->profile_done = true;
            }
            break;
    }
}










