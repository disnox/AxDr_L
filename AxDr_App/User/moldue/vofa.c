#include "common.h"
#include "modlue.h"
#include "usbd_cdc_if.h"

#define MAX_BUFFER_SIZE 1024
uint8_t send_buf[MAX_BUFFER_SIZE];
uint16_t cnt = 0;

/**
***********************************************************************
* @brief:      vofa_start(void)
* @param:	void
* @retval:     void
* @details:    å‘é€
***********************************************************************
**/
float adc_value[3];
extern uint16_t adc1_buff[2];
extern uint16_t adc2_buff[4];
_RAM_FUNC void vofa_start(void)
{
	
//	vofa_send_data(0, pm.ctrl.drag_pe);
//	vofa_send_data(1, pm.foc.p_e);
	vofa_send_data(1, pm.adc.ia);
	vofa_send_data(2, pm.adc.ib);
	vofa_send_data(3, pm.adc.ic);
	
//	vofa_send_data(1, pm.foc.wr_f);
//	vofa_send_data(2, pm.foc.i_q);
//	vofa_send_data(2, pm.foc.i_b);
//	vofa_send_data(3, pm.foc.i_c);

	// calibr
//	vofa_send_data(0, pm.calibr.pos);
//	vofa_send_data(1, pm.calibr.pr_set);
//	vofa_send_data(2, pm.calibr.pos_err);
//	vofa_send_data(3, pm.map.enc_lut);
//	vofa_send_data(3, pm.calibr.pn);
//	vofa_send_data(3, pm.calibr.e_off);
//	vofa_send_data(3, pm.calibr.r_off);
//	vofa_send_data(3, pm.para.phase_order);
	
	// idpm
//	vofa_send_data(0, pm.idpm.Rs);
//	vofa_send_data(2, pm.idpm.Ld);
//	vofa_send_data(3, pm.idpm.Lq);
//	vofa_send_data(5, pm.idpm.flux);
//	vofa_send_data(6, pm.idpm.Js);
//	vofa_send_data(7, pm.foc.i_q);
//	vofa_send_data(8, pm.foc.wr_f);
//	vofa_send_data(9,  pm.id_pi.kp);
//	vofa_send_data(10, pm.id_pi.ki);
//	vofa_send_data(11, pm.iq_pi.kp);
//	vofa_send_data(12, pm.iq_pi.ki);
//	vofa_send_data(13, pm.spd_pi.kp);
//	vofa_send_data(14, pm.spd_pi.ki);

//	vofa_send_data(0, pm.foc.i_d);
//	vofa_send_data(1, pm.ctrl.id_set);
//	vofa_send_data(2, pm.foc.i_q);
//	vofa_send_data(3, pm.ctrl.iq_set);
//	vofa_send_data(4, pm.foc.wr_f);

	
//	vofa_send_data(4, pm.foc.wr_f);
//	vofa_send_data(4, pm.ctrl.wm_set);
//	vofa_send_data(4, pm.traj.spd_step);
//	vofa_send_data(4, pm.foc.sp_m);
//	
//	vofa_send_data(4, pm.foc.mp_m);
//	vofa_send_data(4, pm.ctrl.posm_set);
//	vofa_send_data(4, pm.ctrl.posr_set);
//	
//	vofa_send_data(4, pm.foc.rev);
	
	vofa_sendframetail();
}

/**
***********************************************************************
* @brief:      vofa_transmit(uint8_t* buf, uint16_t len)
* @param:		   void
* @retval:     void
* @details:    ÐÞ¸ÄÍ¨ÐÅ¹¤¾ß£¬USART»òÕßUSB
***********************************************************************
**/
void vofa_transmit(uint8_t* buf, uint16_t len)
{
//	HAL_UART_Transmit(&huart3, (uint8_t *)buf, len, 0xFFFF);
	CDC_Transmit_FS((uint8_t *)buf, len);
}
/**
***********************************************************************
* @brief:      vofa_send_data(float data)
* @param[in]:  num: Êý¾Ý±àºÅ data: Êý¾Ý 
* @retval:     void
* @details:    ½«¸¡µãÊý¾Ý²ð·Ö³Éµ¥×Ö½Ú
***********************************************************************
**/
_RAM_FUNC void vofa_send_data(uint8_t num, float data) 
{
//	send_buf[cnt++] = byte0(data);
//	send_buf[cnt++] = byte1(data);
//	send_buf[cnt++] = byte2(data);
//	send_buf[cnt++] = byte3(data);
	
	data_u f;
	
	f.f_val = data;
	
	send_buf[cnt++] = f.u8_val[0];
	send_buf[cnt++] = f.u8_val[1];
	send_buf[cnt++] = f.u8_val[2];
	send_buf[cnt++] = f.u8_val[3];
}
/**
***********************************************************************
* @brief      vofa_sendframetail(void)
* @param      NULL 
* @retval     void
* @details:   ¸øÊý¾Ý°ü·¢ËÍÖ¡Î²
***********************************************************************
**/
void vofa_sendframetail(void) 
{
	send_buf[cnt++] = 0x00;
	send_buf[cnt++] = 0x00;
	send_buf[cnt++] = 0x80;
	send_buf[cnt++] = 0x7f;
	
	/* ½«Êý¾ÝºÍÖ¡Î²´ò°ü·¢ËÍ */
	vofa_transmit((uint8_t *)send_buf, cnt);
	cnt = 0;// Ã¿´Î·¢ËÍÍêÖ¡Î²¶¼ÐèÒªÇåÁã
}
/**
***********************************************************************
* @brief      vofa_demo(void)
* @param      NULL 
* @retval     void
* @details:   demoÊ¾Àý
***********************************************************************
**/
void vofa_demo(void) 
{
	static float scnt = 0.0f;

	scnt += 0.01f;

	if(scnt >= 360.0f)
		scnt = 0.0f;

	float v1 = scnt;
	float v2 = sin((double)scnt / 180 * 3.14159) * 180 + 180;
	float v3 = sin((double)(scnt + 120) / 180 * 3.14159) * 180 + 180;
	float v4 = sin((double)(scnt + 240) / 180 * 3.14159) * 180 + 180;

	// Call the function to store the data in the buffer
	vofa_send_data(0, v1);
	vofa_send_data(1, v2);
	vofa_send_data(2, v3);
	vofa_send_data(3, v4);

	// Call the function to send the frame tail
	vofa_sendframetail();
}













