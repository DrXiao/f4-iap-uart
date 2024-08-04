#ifndef __IAP_FRAME_T__
#define __IAP_FRAME_T__

enum iap_op {
        IAP_OP_INTERACT,
        IAP_OP_LAUNCH_APP,
        IAP_OP_UPLOAD_APP,
        IAP_OP_UPLOADING,
        IAP_OP_UPLOAD_COMPLETE,
        IAP_OP_ABORT_UPLOAD,
        IAP_OP_UPLOAD_ERR,
        IAP_OP_UPDATE_FLASH,
        IAP_OP_UPDATE_FLASH_RET,
        IAP_OP_UPDATE_FLASH_FINISH,
        IAP_OP_ERR = 0x20,
};

#define IAP_FRAME_DATA_SZ 256
typedef struct {
        uint16_t op;
        uint16_t data_size;
        char data[IAP_FRAME_DATA_SZ];
        uint32_t checksum;
} iap_frame_t;

uint32_t iap_frame_checksum(iap_frame_t *);

typedef struct {
        const char *hint;
        size_t hint_size;
} iap_op_hint_t;

extern iap_op_hint_t iap_op_hint[];

#endif
