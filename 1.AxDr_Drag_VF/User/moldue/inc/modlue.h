#include "main.h"


// Union for different data representations
typedef union
{
	float f_val;
	uint32_t u32_val;
	int32_t i32_val;
	uint8_t u8_val[4];
} data_u;



/* VOFA functions */
void usb_printf(const char *format, ...);
void vofa_start(void);
void vofa_send_data(uint8_t num, float data); 
void vofa_sendframetail(void);
void vofa_demo(void);
void display_foc(void);
