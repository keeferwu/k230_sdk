/* Copyright (c) 2023, Canaan Bright Sight Co., Ltd
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 1. Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 * CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 * INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <sys/mman.h>
#include <stdbool.h>

#include "k_module.h"
#include "k_type.h"
#include "k_vb_comm.h"
#include "k_video_comm.h"
#include "k_sys_comm.h"
#include "mpi_vb_api.h"
#include "mpi_vvi_api.h"
#include "mpi_sys_api.h"
#include "mpi_vo_api.h"
#include "k_vo_comm.h"
#include "k_connector_comm.h"
#include "mpi_connector_api.h"

#define COLOR_ARGB_RED                           0xFFFF0000U
#define COLOR_ARGB_GREEN                         0xFF00FF00U
#define COLOR_ARGB_BLUE                          0xFF0000FFU

#define COLOR_ABGR_RED                           0xFF0000FFU
#define COLOR_ABGR_GREEN                         0xFF00FF00U
#define COLOR_ABGR_BLUE                          0xFFFF0000U

#define COLOR_RGB565_RED                         0xf800
#define COLOR_RGB565_GREEN                       0x400
#define COLOR_RGB565_BLUE                        0x1f

#define PRIVATE_POLL_SZE                        (320 * 240 * 4) + (1024 * 4)
#define PRIVATE_POLL_NUM                        (4)

typedef struct
{
    k_u64 osd_phy_addr;
    void *osd_virt_addr;
    k_pixel_format format;
    k_vo_point offset;
    k_vo_size act_size;
    k_u32 size;
    k_u32 stride;
    k_u8 global_alptha;
} osd_info;

k_u32 g_pool_id;//memory pool id

//Create a private memory pool
int vo_creat_private_poll(void)
{
    k_s32 ret = 0;
    k_vb_config config;
    k_vb_pool_config pool_config;
    k_u32 pool_id;
    memset(&config, 0, sizeof(config));

    config.max_pool_cnt = 10;//Number of cache pools that can be accommodated in the whole system.
    config.comm_pool[0].blk_cnt = 20;
    config.comm_pool[0].blk_size = PRIVATE_POLL_SZE;          // osd0 - 3 argb 320 x 240
    printf("PRIVATE_POLL_SZE = %d\n",PRIVATE_POLL_SZE);
    config.comm_pool[0].mode = VB_REMAP_MODE_NOCACHE;//VB_REMAP_MODE_NOCACHE; 

    ret = kd_mpi_vb_set_config(&config);//Set MPP video cache pool properties
    ret = kd_mpi_vb_init();//Initialize MPP video cache pool

    memset(&pool_config, 0, sizeof(pool_config));
    pool_config.blk_cnt = PRIVATE_POLL_NUM;
    pool_config.blk_size = PRIVATE_POLL_SZE;
    pool_config.mode = VB_REMAP_MODE_NONE;
    //Create a video cache pool and return a valid cache pool ID number.
    pool_id = kd_mpi_vb_create_pool(&pool_config);      // osd0 - 3 argb 320 x 240

    g_pool_id = pool_id;

    return ret;
}
//Calculate the size and stride according to pixel format.
k_u32 vo_creat_osd_test(k_vo_osd osd, osd_info *info)
{
    k_vo_video_osd_attr attr;

    // set attr
    attr.global_alptha = info->global_alptha;

    if (info->format == PIXEL_FORMAT_ABGR_8888 || info->format == PIXEL_FORMAT_ARGB_8888)
    {
        info->size = info->act_size.width  * info->act_size.height * 4;
        info->stride  = info->act_size.width * 4 / 8;
    }
    else if (info->format == PIXEL_FORMAT_RGB_565 || info->format == PIXEL_FORMAT_BGR_565)
    {
        info->size = info->act_size.width  * info->act_size.height * 2;
        info->stride  = info->act_size.width * 2 / 8;
    }
    else if (info->format == PIXEL_FORMAT_RGB_888 || info->format == PIXEL_FORMAT_BGR_888)
    {
        info->size = info->act_size.width  * info->act_size.height * 3;
        info->stride  = info->act_size.width * 3 / 8;
    }
    else if(info->format == PIXEL_FORMAT_ARGB_4444 || info->format == PIXEL_FORMAT_ABGR_4444)
    {
        info->size = info->act_size.width  * info->act_size.height * 2;
        info->stride  = info->act_size.width * 2 / 8;
    }
    else if(info->format == PIXEL_FORMAT_ARGB_1555 || info->format == PIXEL_FORMAT_ABGR_1555)
    {
        info->size = info->act_size.width  * info->act_size.height * 2;
        info->stride  = info->act_size.width * 2 / 8;
    }
    else
    {
        printf("set osd pixel format failed  \n");
    }

    attr.stride = info->stride;
    attr.pixel_format = info->format;
    attr.display_rect = info->offset;
    attr.img_size = info->act_size;
    kd_mpi_vo_set_video_osd_attr(osd, &attr);//Set osd layer properties

    kd_mpi_vo_osd_enable(osd);//open osd layer

    return 0;
}


k_vb_blk_handle vo_insert_frame(k_video_frame_info *vf_info, void **pic_vaddr)
{
    k_u64 phys_addr = 0;
    k_u32 *virt_addr;
    k_vb_blk_handle handle;
    k_s32 size = 0;

    if (vf_info == NULL)
        return K_FALSE;

    if (vf_info->v_frame.pixel_format == PIXEL_FORMAT_ABGR_8888 || vf_info->v_frame.pixel_format == PIXEL_FORMAT_ARGB_8888)
        size = vf_info->v_frame.height * vf_info->v_frame.width * 4;
    else if (vf_info->v_frame.pixel_format == PIXEL_FORMAT_RGB_565 || vf_info->v_frame.pixel_format == PIXEL_FORMAT_BGR_565)
        size = vf_info->v_frame.height * vf_info->v_frame.width * 2;
    else if (vf_info->v_frame.pixel_format == PIXEL_FORMAT_ABGR_4444 || vf_info->v_frame.pixel_format == PIXEL_FORMAT_ARGB_4444)
        size = vf_info->v_frame.height * vf_info->v_frame.width * 2;
    else if (vf_info->v_frame.pixel_format == PIXEL_FORMAT_RGB_888 || vf_info->v_frame.pixel_format == PIXEL_FORMAT_BGR_888)
        size = vf_info->v_frame.height * vf_info->v_frame.width * 3;
    else if (vf_info->v_frame.pixel_format == PIXEL_FORMAT_ARGB_1555 || vf_info->v_frame.pixel_format == PIXEL_FORMAT_ABGR_1555)
        size = vf_info->v_frame.height * vf_info->v_frame.width * 2;
    else if (vf_info->v_frame.pixel_format == PIXEL_FORMAT_YVU_PLANAR_420)
        size = vf_info->v_frame.height * vf_info->v_frame.width * 3 / 2;

    size = size + 4096;         // 强制4K ，后边得删了

    printf("vb block size is %x \n", size);

    handle = kd_mpi_vb_get_block(g_pool_id, size, NULL);//Get a cache block in user mode
    if (handle == VB_INVALID_HANDLE)
    {
        printf("%s get vb block error\n", __func__);
        return K_FAILED;
    }

    phys_addr = kd_mpi_vb_handle_to_phyaddr(handle);//Get block phy Addr
    if (phys_addr == 0)
    {
        printf("%s get phys addr error\n", __func__);
        return K_FAILED;
    }

    virt_addr = (k_u32 *)kd_mpi_sys_mmap(phys_addr, size);//phy addr mmap to vir addr
    // virt_addr = (k_u32 *)kd_mpi_sys_mmap_cached(phys_addr, size);

    if (virt_addr == NULL)
    {
        printf("%s mmap error\n", __func__);
        return K_FAILED;
    }

    vf_info->mod_id = K_ID_VO; //video output device
    vf_info->pool_id = g_pool_id; 
    vf_info->v_frame.phys_addr[0] = phys_addr;
    if (vf_info->v_frame.pixel_format == PIXEL_FORMAT_YVU_PLANAR_420)
        vf_info->v_frame.phys_addr[1] = phys_addr + (vf_info->v_frame.height * vf_info->v_frame.stride[0]);
    *pic_vaddr = virt_addr;

    printf("phys_addr is %lx \n", phys_addr);

    return handle;//返回有效的缓存块句柄
}

void vo_osd_filling_color(osd_info *osd, void *pic_vaddr)
{
    int i = 0;
    k_u32 *temp_addr = (k_u32 *)pic_vaddr;


    if (osd->format == PIXEL_FORMAT_ABGR_8888)
    {
        for (i = 0; i < (osd->size / sizeof(k_u32)) ; i++)
        {
            temp_addr[i] = COLOR_ABGR_RED;
        }
    }
    else if (osd->format == PIXEL_FORMAT_ARGB_8888)
    {
        for (i = 0; i < osd->size / sizeof(k_u32) ; i++)
        {
            //temp_addr[i] = 0x00FF00ff;//COLOR_BGRA_GREEN;
            temp_addr[i] = 0x0000FFFF;//COLOR_BGRA_RED;
            //temp_addr[i] = 0x0000FFFF;//COLOR_BGRA_BLUE;
        }
    }
    else if (osd->format == PIXEL_FORMAT_RGB_565)
    {
        for (i = 0; i < osd->size / sizeof(k_u32); i++)
        {
            temp_addr[i] = 0xFF0000FF;
        }
    }
    else if (osd->format == PIXEL_FORMAT_BGR_565)
    {
        for (i = 0; i < osd->size / sizeof(k_u32); i++)
        {
            temp_addr[i] = 0xFF0000FF;
        }
    }
    else if (osd->format == PIXEL_FORMAT_RGB_888)
    {
        for (i = 0; i < osd->size / sizeof(k_u32); i++)
        {
            temp_addr[i] = 0xFF0000FF;
        }
    }
    else if (osd->format == PIXEL_FORMAT_BGR_888)
    {
        for (i = 0; i < osd->size / sizeof(k_u32); i++)
        {
            temp_addr[i] = 0xFF0000FF;
        }
    }
}
// init lcd connector
k_s32 sample_connector_init(k_connector_type type)
{
    k_u32 ret = 0;
    k_s32 connector_fd;
    k_connector_type connector_type = type;
    k_connector_info connector_info;

    memset(&connector_info, 0, sizeof(k_connector_info));

    // get connector info
    ret = kd_mpi_get_connector_info(connector_type, &connector_info);
    if (ret) {
        printf("sample_vicap, the sensor type not supported!\n");
        return ret;
    }
    //Get Device node of connector
    connector_fd = kd_mpi_connector_open(connector_info.connector_name);
    if (connector_fd < 0) {
        printf("%s, connector open failed.\n", __func__);
        return K_ERR_VO_NOTREADY;
    }

    // set connect power
    kd_mpi_connector_power_set(connector_fd, 1);

    // connector init
    kd_mpi_connector_init(connector_fd, connector_info);
    printf("connector init success!!\n");
    return 0;
}



#define CONNECTOR_OSD_TEST_PICTURE        "disney_320x240_argb8888.yuv"
#define LOAD_PICTURE                       0

k_s32 sample_connector_osd_install_frame(k_connector_type type)
{
    osd_info osd;
    void *pic_vaddr = NULL;
    k_vo_osd osd_id = K_VO_OSD3;
    k_vb_blk_handle block;
    k_video_frame_info vf_info;

    //osd叠加尺寸
    osd.act_size.width = 320 ;
    osd.act_size.height = 240;
    osd.offset.x = 80;
    osd.offset.y = 280;
    osd.global_alptha = 0xff;
    osd.format = PIXEL_FORMAT_ARGB_8888;//PIXEL_FORMAT_ARGB_4444; //PIXEL_FORMAT_ARGB_1555;//PIXEL_FORMAT_ARGB_8888;

    sample_connector_init(type);//LCD INIT

    vo_creat_private_poll();//creat Memery Poll
    // config osd
    vo_creat_osd_test(osd_id, &osd);
    // set frame
    memset(&vf_info, 0, sizeof(vf_info));
    vf_info.v_frame.width = osd.act_size.width;
    vf_info.v_frame.height = osd.act_size.height;
    vf_info.v_frame.stride[0] = osd.act_size.width;
    vf_info.v_frame.pixel_format = osd.format;
    block = vo_insert_frame(&vf_info, &pic_vaddr);//将申请的缓冲区地址给pic_vaddr

#if LOAD_PICTURE
    void *read_addr = NULL;
    FILE *fd;
    int ret = 0;
    k_u32 read_size = osd.size;

    read_addr = malloc(read_size);
    if (!read_addr)
    {
        printf("alloc read addr failed\n");
    }
    // add picture
    fd = fopen(CONNECTOR_OSD_TEST_PICTURE, "rb");
    // get output image
    ret = fread(read_addr, read_size, 1, fd);
    if (ret <= 0)
    {
        printf("fread  picture_addr is failed ret is %d \n", ret);
    }
    memcpy(pic_vaddr, read_addr, read_size);

#else
    vo_osd_filling_color(&osd, pic_vaddr);//填充显示帧
#endif

    kd_mpi_vo_chn_insert_frame(osd_id + 3, &vf_info);  //K_VO_OSD0

    printf("Press Enter to exit \n");

    getchar();

    // close plane
    kd_mpi_vo_osd_disable(osd_id);//close osd layer
    kd_mpi_vb_release_block(block);//release user block
    kd_mpi_vb_destory_pool(g_pool_id);//destory video Memery pool
    kd_mpi_vb_exit();
#if LOAD_PICTURE
    fclose(fd);
#endif
    //exit ;
    return 0;
}

int main(int argc, char *argv[])
{
    // rst display subsystem
    kd_display_reset();
    // set hardware reset;
    kd_display_set_backlight();

    usleep(2000);

    sample_connector_osd_install_frame(ILI9806_MIPI_2LAN_480X800_30FPS);

    return 0;
}
