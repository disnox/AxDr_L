#include "bsp_fdcan.h"
#include "bsp.h"
#include "fdcan.h"

void bsp_can_init(void)
{
	bsp_fdcan_set_baud(fdcan1, CAN_BR_1M, CAN_BR_1M);
	
	bsp_can_filter_init(fdcan1, 0x20);
	HAL_FDCAN_Start(&hfdcan1);                               //开启FDCAN
//	HAL_FDCAN_Start(&hfdcan2);
//	HAL_FDCAN_Start(&hfdcan3);
	HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
	HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO1_NEW_MESSAGE, 0);
}

int8_t bsp_fdcan_port_is(fdcan_num_e comport, FDCAN_HandleTypeDef *hfdcan)
{
	if(comport > fdcan_max) {
		return -1;
	}
	else if(comport == fdcan1) {
		*hfdcan = hfdcan1;
	}
	else if(comport == fdcan2) {
//		*hfdcan = hfdcan2;
	}
	else if(comport == fdcan3) {
//		*hfdcan = hfdcan3;
	}
	else {
		return -1;
	}
	return 0;
}

int8_t bsp_fdcan_start(fdcan_num_e comport)
{
	int8_t ret;
	FDCAN_HandleTypeDef hfdcan;
	
	if(bsp_fdcan_port_is(comport, &hfdcan)==0)
		ret = HAL_FDCAN_Start(&hfdcan);
	return ret;
}


int8_t bsp_fdcan_deinit(fdcan_num_e comport)
{
	FDCAN_HandleTypeDef hfdcan;
	
	if(bsp_fdcan_port_is(comport, &hfdcan)==0)
		HAL_FDCAN_MspDeInit(&hfdcan);
	return 0;
}

//CAN(FD)波特率推荐设置：
//1.BRp应尽量小以保证TG尽可能小，减少误差；
//2.CANFD推荐仲裁域brp<=数据域brp：
//3.SJW应尽量大，尽量保持与TSEG2一致，以提高位宽容忍度：
//4、采样点推荐范围：75~87.5%：
//波特率>800Kbps时，推荐采样点75%：
//波特率>500Kbps时，推荐采样点80%；
//波特率<=500Kbps时推荐采样点87.5%：
//5，应尽量保证总线上所有节点采样点一致，简体中文仲裁域和数据域采样点不要求一致
//6.CANFD仲裁域与数据域波特率之比应大于1/8；

int8_t bsp_fdcan_set_baud(fdcan_num_e comport, uint8_t nominal_baud, uint8_t data_baud)
{
	FDCAN_HandleTypeDef hfdcan;
	int8_t ret = 0;
	uint32_t nom_brp=0, nom_seg1=0, nom_seg2=0, nom_sjw=0;
	uint32_t dat_brp=0, dat_seg1=0, dat_seg2=0, dat_sjw=0;
	
	/*	nominal_baud = 80M/brp/(1+seg1+seg2)
		sample point = (1+seg1)/(1+seg1+seg2)
		sjw :	1-128
		seg1:	2-256
		seg2: 	2-128
		brp :	1-512  */
	switch (nominal_baud)
	{
		case CAN_BR_10K: 	nom_brp=40; nom_seg1=174; nom_seg2=25; nom_sjw=25; break; // sample point 87.5%
		case CAN_BR_20K: 	nom_brp=20; nom_seg1=174; nom_seg2=25; nom_sjw=25; break; // sample point 87.5%
		case CAN_BR_50K: 	nom_brp=8 ; nom_seg1=174; nom_seg2=25; nom_sjw=25; break; // sample point 87.5%
		case CAN_BR_125K: 	nom_brp=4 ; nom_seg1=139; nom_seg2=20; nom_sjw=20; break; // sample point 87.5%
		case CAN_BR_200K: 	nom_brp=2 ; nom_seg1=174; nom_seg2=25; nom_sjw=25; break; // sample point 87.5%
		case CAN_BR_250K: 	nom_brp=2 ; nom_seg1=139; nom_seg2=20; nom_sjw=20; break; // sample point 87.5%
		case CAN_BR_500K: 	nom_brp=1 ; nom_seg1=139; nom_seg2=20; nom_sjw=20; break; // sample point 87.5%
		case CAN_BR_800K: 	nom_brp=1 ; nom_seg1=79 ; nom_seg2=20; nom_sjw=20; break; // sample point 80%
		case CAN_BR_1M:		nom_brp=1 ; nom_seg1=59 ; nom_seg2=20; nom_sjw=20; break; // sample point 75%
	}
	/*	data_baud	 = 80M/brp/(1+seg1+seg2)
		sample point = (1+seg1)/(1+seg1+seg2)
		sjw :	1-16
		seg1:	1-32
		seg2: 	2-16
		brp :	1-32  */
	switch (data_baud)
	{
		case CAN_BR_125K: 	dat_brp=16; dat_seg1=31; dat_seg2=8 ; dat_sjw=8 ; break;	// sample point 80%
		case CAN_BR_200K: 	dat_brp=10; dat_seg1=31; dat_seg2=8 ; dat_sjw=8 ; break;	// sample point 80%
		case CAN_BR_250K: 	dat_brp=20; dat_seg1=13; dat_seg2=2 ; dat_sjw=2 ; break;	// sample point 87.5%
		case CAN_BR_500K: 	dat_brp=6 ; dat_seg1=27; dat_seg2=4 ; dat_sjw=4 ; break;	// sample point 87.5%
		case CAN_BR_800K: 	dat_brp=4 ; dat_seg1=19; dat_seg2=5 ; dat_sjw=5 ; break;	// sample point 80%
		case CAN_BR_1M: 	dat_brp=2 ; dat_seg1=31; dat_seg2=8 ; dat_sjw=8 ; break;	// sample point 80%
		case CAN_BR_2M: 	dat_brp=1 ; dat_seg1=29; dat_seg2=10; dat_sjw=10; break;	// sample point 75%
		case CAN_BR_2M5: 	dat_brp=1 ; dat_seg1=23; dat_seg2=8 ; dat_sjw=8 ; break;	// sample point 75%
		case CAN_BR_3M2: 	dat_brp=1 ; dat_seg1=18; dat_seg2=6 ; dat_sjw=6 ; break;	// sample point 76%
		case CAN_BR_4M: 	dat_brp=1 ; dat_seg1=14; dat_seg2=5 ; dat_sjw=5 ; break;	// sample point 75%
		case CAN_BR_5M:		dat_brp=1 ; dat_seg1=14; dat_seg2=1 ; dat_sjw=1 ; break;	// sample point 93.75%
	}
	
	
	if(bsp_fdcan_port_is(comport, &hfdcan)==0)
	{
		HAL_FDCAN_DeInit(&hfdcan);
		
		hfdcan.Init.NominalPrescaler = nom_brp;
		hfdcan.Init.NominalTimeSeg1  = nom_seg1;
		hfdcan.Init.NominalTimeSeg2  = nom_seg2;
		hfdcan.Init.NominalSyncJumpWidth = nom_sjw;
		
		hfdcan.Init.DataPrescaler = dat_brp;
		hfdcan.Init.DataTimeSeg1  = dat_seg1;
		hfdcan.Init.DataTimeSeg2  = dat_seg2;
		hfdcan.Init.DataSyncJumpWidth = dat_sjw;
		
		ret = HAL_FDCAN_Init(&hfdcan);
	}

	return ret;
}


int8_t bsp_can_filter_init(fdcan_num_e comport, uint16_t filt_id)
{
	FDCAN_HandleTypeDef hfdcan;
	FDCAN_FilterTypeDef fdcan_filter;
	int8_t ret = 0;
	
	fdcan_filter.IdType = FDCAN_STANDARD_ID;                       //标准ID
	fdcan_filter.FilterIndex = 0;                                  //滤波器索引                   
	fdcan_filter.FilterType = FDCAN_FILTER_DUAL;                   //只接收这两个ID
	fdcan_filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;           //过滤器0关联到FIFO0  
	fdcan_filter.FilterID1 = filt_id;
	fdcan_filter.FilterID2 = 0x7FF;
	
	if(bsp_fdcan_port_is(comport, &hfdcan)==0)
	{
		HAL_FDCAN_ConfigFilter(&hfdcan, &fdcan_filter); 		 				  //接收ID2
		//拒绝接收匹配不成功的标准ID和扩展ID,不接受远程帧
		ret = HAL_FDCAN_ConfigGlobalFilter(&hfdcan1,FDCAN_REJECT,FDCAN_REJECT,FDCAN_REJECT_REMOTE,FDCAN_REJECT_REMOTE);
	}
	return ret;
}

int8_t bsp_fdcan_send_data(fdcan_num_e comport, uint16_t id, uint8_t *data, uint8_t len)
{	
	FDCAN_HandleTypeDef hfdcan;
    FDCAN_TxHeaderTypeDef pTxHeader;
	int8_t ret = 0;
	
    pTxHeader.Identifier=id;
    pTxHeader.IdType=FDCAN_EXTENDED_ID;
    pTxHeader.TxFrameType=FDCAN_DATA_FRAME;
	
	if(len<=8)
		pTxHeader.DataLength = len;
	if(len==12)
		pTxHeader.DataLength = FDCAN_DLC_BYTES_12;
	if(len==16)
		pTxHeader.DataLength = FDCAN_DLC_BYTES_16;
	if(len==20)
		pTxHeader.DataLength = FDCAN_DLC_BYTES_20;
	if(len==24)
		pTxHeader.DataLength = FDCAN_DLC_BYTES_24;
	if(len==32)
		pTxHeader.DataLength = FDCAN_DLC_BYTES_32;
	if(len==48)
		pTxHeader.DataLength = FDCAN_DLC_BYTES_48;
	if(len==64)
		pTxHeader.DataLength = FDCAN_DLC_BYTES_64;
	
    pTxHeader.ErrorStateIndicator=FDCAN_ESI_ACTIVE;
    pTxHeader.BitRateSwitch=FDCAN_BRS_ON;
    pTxHeader.FDFormat=FDCAN_FD_CAN;
    pTxHeader.TxEventFifoControl=FDCAN_NO_TX_EVENTS;
    pTxHeader.MessageMarker=0;
	
	if(bsp_fdcan_port_is(comport, &hfdcan)==0)
	{
		ret = HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan, &pTxHeader, data);
	}
	
	return ret;	
}

int8_t bsp_fdcan_receive(fdcan_num_e comport, uint16_t *rec_id, uint8_t *buf, uint8_t *len)
{	
	FDCAN_RxHeaderTypeDef pRxHeader;
	FDCAN_HandleTypeDef hfdcan;
	int8_t ret = 0;
	
	if(bsp_fdcan_port_is(comport, &hfdcan)==0)
	{
		ret = HAL_FDCAN_GetRxMessage(&hfdcan,FDCAN_RX_FIFO0, &pRxHeader, buf);

		if(ret == 0)
		{
			*rec_id = pRxHeader.Identifier;
			if(pRxHeader.DataLength<=FDCAN_DLC_BYTES_8)
				*len = pRxHeader.DataLength;
			if(pRxHeader.DataLength==FDCAN_DLC_BYTES_12)
				*len = 12;
			if(pRxHeader.DataLength==FDCAN_DLC_BYTES_16)
				*len = 16;
			if(pRxHeader.DataLength==FDCAN_DLC_BYTES_20)
				*len = 20;
			if(pRxHeader.DataLength==FDCAN_DLC_BYTES_24)
				*len = 24;
			if(pRxHeader.DataLength==FDCAN_DLC_BYTES_32)
				*len = 32;
			if(pRxHeader.DataLength==FDCAN_DLC_BYTES_48)
				*len = 48;
			if(pRxHeader.DataLength==FDCAN_DLC_BYTES_64)
				*len = 64;
		}
	}
	return ret;	
}


__weak void fdcan1_rx_callback(void)
{

}

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    if (hfdcan == &hfdcan1)
    {
		fdcan1_rx_callback();
    }
}
















