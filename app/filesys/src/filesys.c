#include <stm32f4xx.h>

#include <stdbool.h>
#include <string.h>

#include "filesys.h"

/* File System */
static FRESULT filesys_mount(filesys_t *, const char *, uint8_t);
static FRESULT filesys_unmount(filesys_t *);
static FRESULT filesys_rename(filesys_t *, const char *, const char *);
static FRESULT filesys_rm(filesys_t *, const char *);

void filesys_init(filesys_t *filesys)
{
        *filesys = (filesys_t){.mount = filesys_mount,
                               .unmount = filesys_unmount,
                               .rename = filesys_rename,
                               .rm = filesys_rm};
        uint8_t ret = FATFS_LinkDriver(&SD_Driver, filesys->drive);
        if (ret) {
                Error_Handler();
        }
}

static FRESULT filesys_mount(filesys_t *filesys, const char *path, uint8_t opt)
{
        strcpy(filesys->drive, path);
        return f_mount(&filesys->fs, filesys->drive, opt);
}

static FRESULT filesys_unmount(filesys_t *filesys)
{
#define f_unmount(path) f_mount(0, path, 0);
        return f_unmount(filesys->drive);
}

static FRESULT filesys_rename(filesys_t *filesys,
                              const char *old_name,
                              const char *new_name)
{
        return f_rename(old_name, new_name);
}

static FRESULT filesys_rm(filesys_t *filesys, const char *filename)
{
        return f_unlink(filename);
}


/* File */
static FRESULT file_open(file_t *, const char *, uint8_t);
static FRESULT file_read(file_t *, void *, uint32_t);
static FRESULT file_write(file_t *, const char *, uint32_t);
static FRESULT file_close(file_t *);
static FRESULT file_stat(file_t *);

void file_init(file_t *file)
{
        *file = (file_t){
            .ret_num = 0,
            .open = file_open,
            .read = file_read,
            .write = file_write,
            .close = file_close,
            .stat = file_stat,
        };
}

static FRESULT file_open(file_t *file, const char *path, uint8_t opt)
{
        strcpy(file->path, path);
        return f_open(&file->file, file->path, opt);
}

static FRESULT file_read(file_t *file, void *buf, uint32_t num)
{
        return f_read(&file->file, buf, num, &file->ret_num);
}

static FRESULT file_write(file_t *file, const char *buf, uint32_t num)
{
        return f_write(&file->file, buf, num, &file->ret_num);
}

static FRESULT file_close(file_t *file)
{
        return f_close(&file->file);
}

static FRESULT file_stat(file_t *file)
{
        return f_stat(file->path, &file->info);
}
