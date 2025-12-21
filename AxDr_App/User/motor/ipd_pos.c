//#include "ipd_pos.h"
//#include "common.h"


//ipd_t ipd;

//// 六脉冲IPD 12个脉冲固定角度
//const float ipd_theta[12] = {
//    0.000000f, 3.141592f,  0.523599f, -2.617994f, 1.047198f, -2.094395f,
//    1.570796f, -1.570796f, 2.094395f, -1.047198f, 2.617994f, -0.523599f,
//};

//const float	ipd_theta2[6] = {
//		0.000000f, 3.141592f, 1.047198f, -2.094395f, 2.094395f, -1.047198f,
//};

//// 六脉冲初始角度定位
//void ipd_pos_run(ipd_t *obj, float *vd)
//{
//	static u16 sample_cnt = 0;
//	switch (obj->step)
//    {
//    	case 1:
//			MTR.p_e = ipd_theta2[obj->section];
//			*vd = 0.15f;
//			obj->step = 2;
//    		break;
//    	case 2:
//			sample_cnt++;
//			if(sample_cnt > 10)
//			{
//				*vd = 0.0f;
//				sample_cnt = 0;
//				obj->is[obj->section] = sqrtf(ctrler.ID*ctrler.ID + ctrler.IQ*ctrler.IQ);
//				obj->step = 3;
//			}
//    		break;
//		case 3:
////			if((fabsf(foc.i_d) < 0.05f) && (fabsf(foc.i_q) < 0.05f))
//				sample_cnt++;
//			if(sample_cnt > 30)
//			{
//				sample_cnt = 0;
//				obj->section++;
//				
//				if (obj->section >= 6)
//				{
//					obj->step=4;
//					obj->is_max = obj->is[0];
//					obj->init_pos = ipd_theta2[0];
//					break;
//				}
//				obj->step = 1;
//			}
//			break;
//		case 4:
//			if(obj->is_max < obj->is[sample_cnt])			// 找出最大电流
//			{
//				obj->is_max = obj->is[sample_cnt];
//				obj->init_pos = ipd_theta2[sample_cnt];
//				obj->init_pos = lim_angf(obj->init_pos);
//			}
//			
//			sample_cnt++;
//			if (sample_cnt >= 6)
//			{
//				sample_cnt = 0;
//				obj->step = 6;
//				break;
//			}
//			break;
//			case 6:
//			{
//				sample_cnt = 0;
//				obj->section = 0;
//				obj->step = 0;
//				obj->over_flag = 1;
//				*vd = 0;
//				break;
//			}
//    	default:
//    		break;
//    }
//}


