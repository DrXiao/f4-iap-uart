#ifndef __FILE_SYS_H__
#define __FILE_SYS_H__
#include "fatfs.h"

typedef struct filesys filesys_t;
typedef struct file file_t;

#define FILE_NAME_MAX_LEN 32
struct file {
        FIL file;
        char path[FILE_NAME_MAX_LEN];
        FILINFO info;
        uint32_t ret_num;
        FRESULT (*open)(file_t *, const char *, uint8_t);
        FRESULT (*read)(file_t *, void *, uint32_t);
        FRESULT (*write)(file_t *, const char *, uint32_t);
        FRESULT (*close)(file_t *);
        FRESULT (*stat)(file_t *);
};

void file_init(file_t *);

struct filesys {
        FATFS fs;
        char drive[4];
        FRESULT (*mount)(filesys_t *, const char *, uint8_t);
        FRESULT (*unmount)(filesys_t *);
        FRESULT (*rename)(filesys_t *, const char *, const char *);
        FRESULT (*rm)(filesys_t *, const char *);
};

void filesys_init(filesys_t *);

#endif
