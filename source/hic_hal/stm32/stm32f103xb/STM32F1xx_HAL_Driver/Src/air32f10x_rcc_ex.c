#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <stdint.h>
#include "stm32f1xx_hal.h"

typedef enum 
{
	FLASH_Div_0 = 0,
	FLASH_Div_2 = 1,
	FLASH_Div_4 = 2,
	FLASH_Div_6 = 3,
	FLASH_Div_8 = 4,
	FLASH_Div_16 = 5,
}FlashClkDiv;

#define SysFreq_Set		(*((void (*)(uint32_t, FlashClkDiv , uint8_t, uint8_t))(*(uint32_t *)0x1FFFD00C)))

uint32_t AIR_RCC_PLLConfig(uint32_t RCC_PLLSource, uint32_t RCC_PLLMul, uint8_t Latency)
{	
	volatile uint32_t sramsize = 0;
	uint32_t pllmul = 0;
	FunctionalState pwr_gating_state = 0;
	/* Check the parameters */
	assert_param(IS_RCC_PLL_SOURCE(RCC_PLLSource));
	assert_param(IS_RCC_PLL_MUL(RCC_PLLMul));
	
	*(volatile uint32_t *)(0x400210F0) = BIT(0);//开启sys_cfg门控
	*(volatile uint32_t *)(0x40016C00) = 0xa7d93a86;//解一、二、三级锁
	*(volatile uint32_t *)(0x40016C00) = 0xab12dfcd;
	*(volatile uint32_t *)(0x40016C00) = 0xcded3526;
	sramsize = *(volatile uint32_t *)(0x40016C18);
	*(volatile uint32_t *)(0x40016C18) = 0x200183FF;//配置sram大小, 将BOOT使用对sram打开
	*(volatile uint32_t *)(0x4002228C) = 0xa5a5a5a5;//QSPI解锁
	
	SysFreq_Set(RCC_PLLMul,Latency ,0,1);
	RCC->CFGR = (RCC->CFGR & ~0x00030000) | RCC_PLLSource;
	
	//恢复配置前状态
	*(volatile uint32_t *)(0x40016C18) = sramsize;
	*(volatile uint32_t *)(0x400210F0) = 0;//开启sys_cfg门控
	*(volatile uint32_t *)(0x40016C00) = ~0xa7d93a86;//加一、二、三级锁
	*(volatile uint32_t *)(0x40016C00) = ~0xab12dfcd;
	*(volatile uint32_t *)(0x40016C00) = ~0xcded3526;
	*(volatile uint32_t *)(0x4002228C) = ~0xa5a5a5a5;//QSPI解锁
	
	
	return 1;
}

#include <sys/stat.h>
#include <unistd.h>

int _close(int fd) {
    return -1;
}

int _lseek(int fd, int ptr, int dir) {
    return 0;
}

int _read(int fd, char *buf, int count) {
    return -1;
}
extern int32_t USBD_CDC_ACM_DataSend(const uint8_t *buf, int32_t len);
extern void main_blink_cdc_led(int led_mode);
int _write(int fd, const char *buf, int count) {
	USBD_CDC_ACM_DataSend(buf , count);
	main_blink_cdc_led(1);
    return count;
}

int _fstat(int fd, struct stat *st) {
    st->st_mode = S_IFCHR;
    return 0;
}

int _isatty(int fd) {
    return 1;
}

int _open(const char *name, int flags, int mode) {
    return -1;
}

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include "DAP_config.h"

/**
 * 仿真器在 TCK 下降沿改变 TMS、TDI 电平，被测芯片在 TCK 上升沿捕获输入并更新内部状态。
 * 被测芯片在 TCK 下降沿改变 TDO 电平，仿真器在 TCK 上升沿捕获 TDO 数据。
 */
/*******************************private func*******************************/
#define JTAG_CYCLE_TMS(tms) \
    do                      \
    {                       \
        iPIN_TMS_OUT(tms);   \
        iPIN_TCK_SET();      \
        iPIN_TCK_CLR();      \
    } while (0);

static void JTAG_sequence_escape(uint32_t n)
{
    bool tms = 1;
    iPIN_TCK_SET();
    for (size_t i = 0; i < n; i++)
    {
        iPIN_TMS_OUT(tms);
        tms = !tms;
    }
    iPIN_TMS_OUT(1);
    iPIN_TCK_CLR();
}

static void JTAG_sequence_tms(uint32_t val, uint32_t bit_len)
{
    for (size_t i = 0; i < bit_len; i++)
    {
        bool tms = val >> i & 1;
        JTAG_CYCLE_TMS(tms);
    }
}
/*******************************interface*******************************/
void cJTAG_seq(uint32_t bits, uint8_t *ucTDI, uint8_t *ucTDO)
{
    bool tms, tdi, tdo;
    uint8_t tmp = 0;
    uint32_t cycle = (bits + 7) / 8;
    uint32_t last_count = bits - (8 * (cycle - 1));
    for (size_t i = 1; i <= cycle; i++)
    {
        tmp = 0;
        for (size_t j = 0; j < (i == cycle ? last_count : 8); j++)
        {
            tms = ((i == cycle) && (j == (last_count-1)))? 1 : 0;
            tdi = (*ucTDI >> j) & 1;
            JTAG_CYCLE_TCK_FAST(tms, tdi, tdo);
            // printf("%d ", tdo);
            tmp |= tdo << j;
        }
        ucTDI++;
        *ucTDO++ = tmp;
    }
}

void cJTAG_tms(uint32_t bits, uint8_t* ucTMS)
{
	// printf("cJTAG_tms: count = %d\n", bits);
    bool tms, tdo;
    for (size_t i = 0; i < bits; i++) {
        tms = (ucTMS[i/8] >> (i%8)) & 1;
        
    USART2->DR = 0x11;
    while((USART2->SR & USART_SR_TXE) == 0);

    JTAG_CYCLE_TCK_FAST(tms, 1, tdo);
    
    USART2->DR = 0x22;
    while((USART2->SR & USART_SR_TXE) == 0);
    }
    (void)tdo;
}

void cJtag_active(void)
{
    JTAG_sequence_escape(10);
    JTAG_sequence_tms(0xFFFF, 24);
    JTAG_sequence_escape(7);
    JTAG_sequence_tms(0x08C, 12);
}