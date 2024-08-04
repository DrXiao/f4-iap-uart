#ifndef __IAP_UART_H__
#define __IAP_UART_H__
#include <stm32f4xx.h>

#include <stdbool.h>
#include <stdint.h>

typedef struct iap_uart_config iap_uart_config_t;
typedef struct chip_config chip_config_t;
typedef struct iap iap_t;

struct iap_uart_config {
        UART_HandleTypeDef *huart;
        volatile bool *recv_flag;
        volatile bool *send_flag;
        volatile uint32_t *recv_remaining_cnt;
        uint32_t recv_cnt;
};

enum chip_family { CHIP_FAMILY_F40x, CHIP_FAMILY_F41x };

struct chip_config {
        enum chip_family chip_family;
        uint32_t chip_flash_addr;
        uint32_t chip_flash_size;
        uint32_t chip_ram_addr;
        uint32_t chip_ram_size;
};

struct iap {
        iap_uart_config_t iap_uart_config;
        chip_config_t chip_config;
        uint32_t iap_max_code_size;
        void (*iap_proc)(struct iap *);
        void (*app_nvic_vec_ofs)(struct iap *);
};

void iap_init(iap_t *, iap_uart_config_t *, chip_config_t *);

#endif
