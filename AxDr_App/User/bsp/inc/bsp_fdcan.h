#ifndef __BSP_FDCAN_H__
#define __BSP_FDCAN_H__

#include "bsp.h"


/************** FDCAN&CAN **************/
#define CAN_BR_10K  0	// 10K 	bps
#define CAN_BR_20K  1	// 20K 	bps
#define CAN_BR_50K  2	// 50K 	bps
#define CAN_BR_125K 3	// 125K bps
#define CAN_BR_200K	4	// 200K bps
#define CAN_BR_250K 5	// 250K bps
#define CAN_BR_500K 6	// 500K bps
#define CAN_BR_800K 7	// 800K bps
#define CAN_BR_1M 	8	// 1M	bps
#define CAN_BR_2M 	9	// 2M	bps
#define CAN_BR_2M5 	10	// 2M5	bps
#define CAN_BR_3M2 	11	// 3M2	bps
#define CAN_BR_4M 	12	// 4M	bps
#define CAN_BR_5M	13	// 5M	bps


typedef enum {
  fdcan1 =0,
  fdcan2,
  fdcan3,
  fdcan_max,
}fdcan_num_e; 


void bsp_can_init(void);
int8_t bsp_can_filter_init(fdcan_num_e comport, uint16_t filt_id);
int8_t bsp_fdcan_set_baud(fdcan_num_e comport, uint8_t nominal_baud, uint8_t data_baud);
int8_t bsp_fdcan_send_data(fdcan_num_e comport, uint16_t id, uint8_t *data, uint8_t len);
int8_t bsp_fdcan_receive(fdcan_num_e comport, uint16_t *rec_id, uint8_t *buf, uint8_t *len);
void fdcan1_rx_callback(void);
void fdcan2_rx_callback(void);
void fdcan3_rx_callback(void);



#endif /* __BSP_FDCAN_H__ */


