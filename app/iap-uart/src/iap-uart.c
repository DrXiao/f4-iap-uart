#include <stm32f4xx.h>

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "filesys.h"
#include "iap-frame.h"
#include "iap-uart.h"

typedef void (*app_type)(void);

static filesys_t filesys;

volatile bool iap_uart_recv_flag;
volatile bool iap_uart_send_flag;
volatile uint32_t iap_uart_recv_remaining_cnt;

void (*chip_erase_flash)(uint32_t, uint32_t);

#define IAP_MAX_CODE_SIZE 0x10000
#define FIRM_BIN_NAME "firmware.bin"
#define TEMP_BIN_NAME "temp.bin"
#define IAP_WAIT_TIME_MS (5 * 1000)

#define iap_uart_send(iap, iap_frame)                             \
        do {                                                      \
                *iap->iap_uart_config.send_flag = false;          \
                HAL_UART_Transmit_DMA(iap->iap_uart_config.huart, \
                                      (uint8_t *) &iap_frame,     \
                                      sizeof(iap_frame));         \
        } while (0)

#define iap_uart_send_sync(iap)                          \
        do {                                             \
                while (!*iap->iap_uart_config.send_flag) \
                        ;                                \
        } while (0)

#define iap_uart_recv(iap, iap_frame)                            \
        do {                                                     \
                *iap->iap_uart_config.recv_flag = false;         \
                __HAL_UART_ENABLE_IT(iap->iap_uart_config.huart, \
                                     UART_IT_IDLE);              \
                HAL_UART_Receive_DMA(iap->iap_uart_config.huart, \
                                     (uint8_t *) &iap_frame,     \
                                     sizeof(iap_frame));         \
        } while (0)

#define iap_uart_recv_sync(iap)                                   \
        do {                                                      \
                while (!*iap->iap_uart_config.recv_flag)          \
                        ;                                         \
                __HAL_UART_DISABLE_IT(iap->iap_uart_config.huart, \
                                      UART_IT_IDLE);              \
                iap->iap_uart_config.recv_cnt =                   \
                    sizeof(iap_frame_t) -                         \
                    *iap->iap_uart_config.recv_remaining_cnt;     \
        } while (0)

#define iap_uart_recv_timeout(iap, iap_frame, timeout)                \
        do {                                                          \
                iap_uart_recv(iap, iap_frame);                        \
                uint32_t curr_time = HAL_GetTick();                   \
                while ((HAL_GetTick() - curr_time) < timeout &&       \
                       !*iap->iap_uart_config.recv_flag)              \
                        ;                                             \
                __HAL_UART_DISABLE_IT(iap->iap_uart_config.huart,     \
                                      UART_IT_IDLE);                  \
                if (*iap->iap_uart_config.recv_flag) {                \
                        iap->iap_uart_config.recv_cnt =               \
                            sizeof(iap_frame_t) -                     \
                            *iap->iap_uart_config.recv_remaining_cnt; \
                }                                                     \
        } while (0)

static void iap_proc(iap_t *);
static void app_nvic_vec_offset(iap_t *);
static void iap_interact(iap_t *);
static void iap_update_flash(iap_t *, iap_frame_t *);
static void f40x_f41x_erase_flash(uint32_t, uint32_t);

void iap_init(iap_t *iap,
              iap_uart_config_t *iap_uart_config,
              chip_config_t *chip_config)
{
        *iap = (iap_t){.iap_uart_config = *iap_uart_config,
                       .chip_config = *chip_config,
                       .iap_max_code_size = IAP_MAX_CODE_SIZE,
                       .iap_proc = iap_proc,
                       .app_nvic_vec_ofs = app_nvic_vec_offset};

        switch (iap->chip_config.chip_family) {
        case CHIP_FAMILY_F40x:
        case CHIP_FAMILY_F41x:
                chip_erase_flash = f40x_f41x_erase_flash;
                break;
        default:
                Error_Handler();
                break;
        }
}

static void iap_proc(iap_t *iap)
{
        uint32_t app_addr =
            iap->chip_config.chip_flash_addr + iap->iap_max_code_size;

        iap_interact(iap);

        if (((*(__IO uint32_t *) (app_addr))) ==
            (iap->chip_config.chip_ram_addr + iap->chip_config.chip_ram_size)) {
                uint32_t jump_addr = *(__IO uint32_t *) (app_addr + 4);
                app_type app = (app_type) jump_addr;
                __set_MSP(*(__IO uint32_t *) app_addr);
                app();
        }

        Error_Handler();
}

static void app_nvic_vec_offset(iap_t *iap)
{
        __set_FAULTMASK(1);
        SCB->VTOR = FLASH_BASE | iap->iap_max_code_size;
        __set_FAULTMASK(0);
}

static void iap_handle_upload(iap_t *iap,
                              iap_frame_t *iap_recv,
                              iap_frame_t *iap_send)
{
        file_t file;
        file_init(&file);
        file.open(&file, TEMP_BIN_NAME, FA_CREATE_ALWAYS | FA_WRITE | FA_READ);
        uint32_t checksum;
        iap_op_hint_t *hint = iap_op_hint;
        while (true) {
                iap_uart_recv(iap, (*iap_recv));
                iap_uart_recv_sync(iap);
                if (sizeof(iap_frame_t) != iap->iap_uart_config.recv_cnt)
                        goto UPLOAD_ERR_HANDLE;

                checksum = iap_frame_checksum(iap_recv);
                if (checksum != iap_recv->checksum)
                        goto UPLOAD_ERR_HANDLE;

                switch (iap_recv->op) {
                case IAP_OP_UPLOADING:
                        iap_send->op = IAP_OP_UPLOADING;
                        file.write(&file, iap_recv->data, iap_recv->data_size);
                        break;
                case IAP_OP_UPLOAD_COMPLETE:
                        iap_send->op = IAP_OP_UPLOAD_COMPLETE;
                        file.close(&file);
                        filesys.rm(&filesys, FIRM_BIN_NAME);
                        filesys.rename(&filesys, TEMP_BIN_NAME, FIRM_BIN_NAME);
                        break;
                case IAP_OP_ABORT_UPLOAD:
                        iap_send->op = IAP_OP_ABORT_UPLOAD;
                        file.close(&file);
                        break;
                default:
                UPLOAD_ERR_HANDLE:
                        iap_send->op = IAP_OP_UPLOAD_ERR;
                }
                hint = &iap_op_hint[iap_send->op];
                iap_send->data_size = hint->hint_size;
                memcpy(iap_send->data, hint->hint, hint->hint_size);
                iap_send->checksum = iap_frame_checksum(iap_send);
                iap_uart_send(iap, (*iap_send));
                iap_uart_send_sync(iap);
                switch (iap_recv->op) {
                case IAP_OP_UPLOAD_COMPLETE:
                case IAP_OP_ABORT_UPLOAD:
                        goto UPLOAD_END;
                }
        }
UPLOAD_END:
        return;
}

static void iap_interact(iap_t *iap)
{
        iap_frame_t iap_recv;
        iap_frame_t iap_send;
        uint32_t checksum;
        iap_op_hint_t *hint;

RECV_INTERACT:
        iap_uart_recv_timeout(iap, iap_recv, IAP_WAIT_TIME_MS);

        if (!*iap->iap_uart_config.recv_flag) {
                goto END_HANDLE;
        }

        checksum = iap_frame_checksum(&iap_recv);
        if (checksum != iap_recv.checksum || IAP_OP_INTERACT != iap_recv.op) {
                iap_send.op = IAP_OP_ERR;
                iap_send.data_size = iap_op_hint[IAP_OP_ERR].hint_size;
                memcpy(iap_send.data, iap_op_hint[IAP_OP_ERR].hint,
                       iap_op_hint[IAP_OP_ERR].hint_size);
                iap_send.checksum = iap_frame_checksum(&iap_send);
                iap_uart_send(iap, iap_send);
                goto RECV_INTERACT;
        }
        filesys_init(&filesys);
        filesys.mount(&filesys, "", 1);

        goto INTERACT_HANDLE;
        while (true) {
                iap_uart_recv(iap, iap_recv);
                iap_uart_recv_sync(iap);
                if (iap->iap_uart_config.recv_cnt != sizeof(iap_frame_t))
                        goto ERR_HANDLE;

                checksum = iap_frame_checksum(&iap_recv);
                if (checksum != iap_recv.checksum)
                        goto ERR_HANDLE;

                switch (iap_recv.op) {
                INTERACT_HANDLE:
                case IAP_OP_INTERACT:
                case IAP_OP_LAUNCH_APP:
                case IAP_OP_UPLOAD_APP:
                case IAP_OP_UPDATE_FLASH:
                        iap_send.op = iap_recv.op;
                        break;
                default:
                ERR_HANDLE:
                        iap_send.op = IAP_OP_ERR;
                        break;
                }

                hint = &iap_op_hint[iap_send.op];
                iap_send.data_size = hint->hint_size;
                memcpy(iap_send.data, hint->hint, hint->hint_size);
                iap_send.checksum = iap_frame_checksum(&iap_send);
                iap_uart_send(iap, iap_send);
                iap_uart_send_sync(iap);

                switch (iap_send.op) {
                case IAP_OP_LAUNCH_APP:
                        filesys.unmount(&filesys);
                        goto END_HANDLE;
                case IAP_OP_UPLOAD_APP:
                        iap_handle_upload(iap, &iap_recv, &iap_send);
                        break;
                case IAP_OP_UPDATE_FLASH:
                        iap_update_flash(iap, &iap_send);
                        break;
                }
        }
END_HANDLE:
        return;
}

static void iap_update_flash(iap_t *iap, iap_frame_t *iap_send)
{
#define IAP_UPDATE_SIZE 2048

        chip_erase_flash(iap->chip_config.chip_flash_size,
                         iap->iap_max_code_size);

        uint32_t addr =
            iap->chip_config.chip_flash_addr + iap->iap_max_code_size;
        file_t file;
        uint8_t bin[IAP_UPDATE_SIZE];
        uint16_t read_size;
        uint32_t file_size;
        uint32_t write_size = 0;
        iap_send->op = IAP_OP_UPDATE_FLASH_RET;

        HAL_FLASH_Unlock();
        file_init(&file);
        file.open(&file, FIRM_BIN_NAME, FA_READ);
        file.stat(&file);
        file_size = file.info.fsize;
        goto RET_PROCESS;

        while (true) {
                file.read(&file, bin, IAP_UPDATE_SIZE);
                read_size = file.ret_num;
                for (uint32_t *ptr = bin; ptr < bin + read_size; ptr++) {
                        if (addr < (iap->chip_config.chip_flash_addr +
                                    iap->chip_config.chip_flash_size)) {
                                if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,
                                                      addr, *ptr) == HAL_OK) {
                                        addr += 4;
                                } else {
                                        HAL_FLASH_Lock();
                                        Error_Handler();
                                }
                        }
                }
                write_size += read_size;
        RET_PROCESS:
                *(float *) iap_send->data =
                    (float) write_size / (float) file_size;
                iap_send->data_size = sizeof(float);
                iap_send->checksum = iap_frame_checksum(iap_send);
                iap_uart_send(iap, *iap_send);
                iap_uart_send_sync(iap);
                if (write_size == file_size)
                        break;
                else if (write_size > file_size)
                        Error_Handler();
        }
        iap_send->op = IAP_OP_UPDATE_FLASH_FINISH;
        iap_send->data_size = iap_op_hint[iap_send->op].hint_size;
        memcpy(iap_send->data, iap_op_hint[iap_send->op].hint,
               iap_send->data_size);
        iap_send->checksum = iap_frame_checksum(iap_send);
        iap_uart_send(iap, *iap_send);
        iap_uart_send_sync(iap);

        file.close(&file);
        HAL_FLASH_Lock();
}

#define KBYTES(sz) (sz * 1024)
static void f40x_f41x_erase_flash(uint32_t flash_size, uint32_t iap_code_size)
{
        uint32_t sectors_size[] = {KBYTES(16),  KBYTES(16),  KBYTES(16),
                                   KBYTES(16),  KBYTES(64),  KBYTES(128),
                                   KBYTES(128), KBYTES(128), KBYTES(128),
                                   KBYTES(128), KBYTES(128), KBYTES(128)};
        uint32_t nb_sectors = 0;
        uint32_t erase_size = flash_size - iap_code_size;
        uint32_t accumulated_size = 0;
        uint32_t sector_err;

        int i = 4;
        while (accumulated_size < erase_size) {
                accumulated_size += sectors_size[i++];
                nb_sectors++;
        }

        FLASH_EraseInitTypeDef erase_init = {
            .TypeErase = FLASH_TYPEERASE_SECTORS,
            .Sector = FLASH_SECTOR_4,
            .NbSectors = nb_sectors};
        HAL_FLASH_Unlock();
        if (HAL_FLASHEx_Erase(&erase_init, &sector_err) != HAL_OK) {
                HAL_FLASH_Lock();
                Error_Handler();
        }
        HAL_FLASH_Lock();
}
