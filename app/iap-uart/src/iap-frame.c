#include <stdint.h>
#include <string.h>

#include "iap-frame.h"

uint32_t iap_frame_checksum(iap_frame_t *frame)
{
        uint32_t checksum = 0;
        const uint8_t *ptr = (uint8_t *) frame;
        for (int i = 0; i < sizeof(*frame) - sizeof(frame->checksum); i++) {
                checksum += *ptr++;
        }
        return checksum;
}

#define ASSIGN_HINT(str)                              \
        {                                             \
                .hint = str, .hint_size = sizeof(str) \
        }
iap_op_hint_t iap_op_hint[] = {
    [IAP_OP_INTERACT] = ASSIGN_HINT("IAP - interactive mode"),
    [IAP_OP_LAUNCH_APP] = ASSIGN_HINT("IAP - launch app"),
    [IAP_OP_UPLOAD_APP] = ASSIGN_HINT("IAP - upload mode"),
    [IAP_OP_UPLOADING] = ASSIGN_HINT("IAP - uploading the image..."),
    [IAP_OP_UPLOAD_COMPLETE] = ASSIGN_HINT("IAP - finish upload"),
    [IAP_OP_ABORT_UPLOAD] = ASSIGN_HINT("IAP - abort uploading image"),
    [IAP_OP_UPLOAD_ERR] = ASSIGN_HINT("IAP - upload mode - error"),
    [IAP_OP_UPDATE_FLASH] = ASSIGN_HINT("IAP - update flash mode"),
    [IAP_OP_UPDATE_FLASH_FINISH] = ASSIGN_HINT("IAP - finish flash update"),
    [IAP_OP_ERR] = ASSIGN_HINT("IAP - error message")};
