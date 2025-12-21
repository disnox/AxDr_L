#include "common.h"
#include "spi_bsp.h"
//#include "spi.h"

void encoder_init(void)
{
    pm.pos_box.ma732.dir = 1;
    pm.pos_box.ma732.bit = 14;
    pm.pos_box.ma732.cpr = 16384;

    pm.pos_box.ma732.shift_bit = log2f(pm.pos_box.ma732.cpr / 256);
    pm.pos_box.ma732.factor = M_2PI /  pm.pos_box.ma732.cpr;

    pm.pos_box.dm485enc.dir = -1;
    pm.pos_box.dm485enc.bit = 17;
    pm.pos_box.dm485enc.cpr = 131072;

    pm.pos_box.dm485enc.shift_bit = log2f(pm.pos_box.dm485enc.cpr / 256);
    pm.pos_box.dm485enc.factor  = M_2PI / pm.pos_box.dm485enc.cpr;
	
	pm.pos_box.mt6816.dir = 1;
    pm.pos_box.mt6816.bit = 14;
    pm.pos_box.mt6816.cpr = 16384;

    pm.pos_box.mt6816.shift_bit = log2f(pm.pos_box.mt6816.cpr / 256);
    pm.pos_box.mt6816.factor = M_2PI /  pm.pos_box.mt6816.cpr;
}

_RAM_FUNC uint8_t encoder_parity(uint16_t v)
{
    uint8_t h_count = 0;
    for (uint8_t j = 0; j < 16; j++)
    {
        if (v & (0x0001 << j))
            h_count++;
    }
    return h_count;
}

_RAM_FUNC uint32_t read_mt6825_raw(void)
{
    uint16_t timeOut = 1000;

    uint16_t tx_data[2] = {0x83ff, 0xffff};
    uint16_t rx_data[2];

    //cs_down;
    //spi_transmit_receive_sync(&hspi1, tx_data[0], &rx_data[0], 200);
    //spi_transmit_receive_sync(&hspi1, tx_data[1], &rx_data[1], 200);
    //cs_up;

    //while( hspi1.State == HAL_SPI_STATE_BUSY ) {
    //  if (timeOut-- ==0) return 0;
    //}   // wait for transmission complete

    uint16_t parity = (rx_data[0] << 8 | (rx_data[1] >> 8));
    uint8_t nomag = ((rx_data[1] >> 8) & 0x02) >> 1;
    if ((encoder_parity(parity) & 0x01) || nomag)
    {
        //		goto done;
    }

    uint32_t raw = ((rx_data[0] & 0x00FF) << 10) | ((rx_data[1] & 0xFC00) >> 6) | ((rx_data[1] & 0x00F0) >> 4);
    return raw;
}

_RAM_FUNC uint32_t read_ma732_raw(void)
{
    uint16_t tx[2] = {0x0000};
    uint16_t rx[2] = {0x0000};
    cs_down;
    spi_transmit_receive_sync(&hspi1, tx[0], &rx, 200);
    //	HAL_SPI_TransmitReceive(&hspi1, (uint8_t*)&tx_data, (uint8_t*)&rx_data, 1, 1000);
    cs_up;
    pm.pos_box.ma732.rev_flag = 1;
    pm.pos_box.ma732.raw = (rx[0] >> 2);
    return 0;
}

_RAM_FUNC uint32_t read_mt6816_raw(void)
{
    uint16_t data_t[2];
    uint16_t data_r[2];
    uint32_t count;
    data_t[0] = (0x80 | 0x03) << 8;
    data_t[1] = (0x80 | 0x04) << 8;

    cs_down;
	spi_transmit_receive_sync(&hspi1 ,data_t[0], &data_r[0], 200);
//    HAL_SPI_TransmitReceive(&hspi1, (uint8_t*)&data_t[0], (uint8_t*)&data_r[0], 1, 1000);
    cs_up;
    cs_down;
	spi_transmit_receive_sync(&hspi1 ,data_t[1], &data_r[1], 200);
//    HAL_SPI_TransmitReceive(&hspi1, (uint8_t*)&data_t[1], (uint8_t*)&data_r[1], 1, 1000);
    cs_up;

    count = (((data_r[0] & 0x00FF) << 8) | (data_r[1] & 0x00FF)) >> 2;
	
	pm.pos_box.mt6816.rev_flag = 1;
    pm.pos_box.mt6816.raw = count;
    return count;
}

_RAM_FUNC uint32_t read_dm485enc_raw(void)
{
    //sys_freq = clock_get_frequency(clock_cpu1);
    //tickinlus= sys_freq*0.000001f;
    //write_csr(CSR_MCYCLE, 0);
    // 1. 发送读取指令
    // tx_dma_buff[0] = 0x10;
    // bsp_uart8_transmit(tx_dma_buff, 1);
    //
    // return 0;
}

_RAM_FUNC uint32_t send_mod_dm485enc(void)
{
    // // 1. 发送调制指令
    // tx_dma_buff[0] = 0x15;
    // tx_dma_buff[1] = 0x24;
    // tx_dma_buff[2] = 0x33;
    // tx_dma_buff[3] = 0x33;
    // tx_dma_buff[4] = 0x42;
    // tx_dma_buff[5] = 0x51;
    // bsp_uart8_transmit(tx_dma_buff, 6);
    //
    // return 0;
}

_RAM_FUNC void pos_encoder_calc(enc_para_t* x, pmsm_t *pm)
{
    uint8_t index;
    float off_1, off_2, off_interp;

    index = x->raw >> x->shift_bit;
//    off_1 = x->lut[index];
//    index++;
//    off_2 = x->lut[index];
    // float frac = (float)(x->raw & ((1 << x->shift_bit) - 1)) / (1 << x->shift_bit);
    off_interp = 0.0f;//off_1 + (off_2 - off_1) * frac;
    x->pos = (x->raw + off_interp) * x->factor;
    wrap_0_2pi(x->pos);
}

_RAM_FUNC void bsp_uart8_rxidle_isr(void)
{
    // if(pm.mode.sys == calibrat_mode && (pm.mode.calibrat == rotor_enc_mod||pm.mode.calibrat == output_enc_mod)) {
    //     pm.modenc.result = (rx_dma_buff[1] << 8) | rx_dma_buff[0];
    // } else {
    //     // uint8_t crc = bsp_crc8_cal(rx_dma_buff, 3);
    //     //uint8_t crc = bsp_soft_crc8_calc(rx_dma_buff, 3);
    //     //if (crc == rx_dma_buff[3]) {
    //         pm.pos_box.dm485enc.rev_flag = 1; // 设置标志位，表示接收到新数据
    //         pm.pos_box.dm485enc.raw = (rx_dma_buff[2] << 16) | (rx_dma_buff[1] << 8) | (rx_dma_buff[0]);
    //     //}
    // }
    //  //run_tick = read_csr(CSR_MCYCLE);
    //  //run_us = run_tick/tickinlus;
    // memset(rx_dma_buff, 0, Uart_BUFF_SIZE);
    // bsp_uart8_receive(rx_dma_buff, Uart_BUFF_SIZE);
}
