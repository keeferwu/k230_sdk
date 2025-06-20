#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <sys/mman.h>
#include <errno.h>
#include <fcntl.h>
#include <stdint.h>

#include "k_module.h"
#include "k_type.h"
#include "k_vb_comm.h"
#include "k_vicap_comm.h"
#include "k_video_comm.h"
#include "k_sys_comm.h"
#include "mpi_vb_api.h"
#include "mpi_vicap_api.h"
#include "mpi_isp_api.h"
#include "mpi_sys_api.h"

#include "mpi_sensor_api.h"

#define VICAP_OUTPUT_BUF_NUM 6
#define VICAP_INPUT_BUF_NUM 4

#define DISPLAY_WITDH  1088
#define DISPLAY_HEIGHT 1920

typedef struct {
    k_vicap_dev dev_num;
    k_bool dev_enable;
    k_vicap_sensor_type sensor_type;
    k_vicap_sensor_info sensor_info;

    k_u16 in_width;
    k_u16 in_height;

    //for mcm
    k_vicap_work_mode mode;
    k_u32 in_size;
    k_pixel_format in_format;

    k_vicap_input_type input_type;
    k_vicap_image_pattern pattern;
    const char *file_path;//input raw image file
    const char *calib_file;
    void *image_data;
    k_u32 dalign;

    k_bool ae_enable;
    k_bool awb_enable;
    k_bool dnr3_enable;
    k_bool hdr_enable;

    k_vicap_chn chn_num[VICAP_CHN_ID_MAX];

    k_bool chn_enable[VICAP_CHN_ID_MAX];
    k_pixel_format out_format[VICAP_CHN_ID_MAX];
    
    k_bool crop_enable[VICAP_CHN_ID_MAX];

    k_vicap_window out_win[VICAP_CHN_ID_MAX];

    k_vicap_window crop_win[VICAP_CHN_ID_MAX];

    k_u32 buf_size[VICAP_CHN_ID_MAX];

    k_video_frame_info dump_info[VICAP_CHN_ID_MAX];

    k_bool preview[VICAP_CHN_ID_MAX];
    k_u16 rotation[VICAP_CHN_ID_MAX];
    k_u8 fps[VICAP_CHN_ID_MAX];
    k_bool dw_enable;
    k_vicap_mirror sensor_mirror;
} vicap_device_obj;

//初始化视频缓存区
static k_s32 sample_vicap_vb_init(vicap_device_obj *dev_obj)
{
    k_s32 ret = 0;
    k_vb_config config;
    k_vb_supplement_config supplement_config;

    memset(&config, 0, sizeof(config));
    config.max_pool_cnt = 64;

    int k = 0;
    int i = 0;
        printf("%s, enable dev(%d)\n", __func__, i);

    int j = 0;
    printf("%s, enable chn(%d), k(%d)\n", __func__, j, k);
    config.comm_pool[k].blk_cnt = VICAP_OUTPUT_BUF_NUM; //缓冲区数量
    config.comm_pool[k].mode = VB_REMAP_MODE_NOCACHE; //无缓存重映射

    k_u16 out_width = dev_obj[i].out_win[j].width; //输出宽度
    k_u16 out_height = dev_obj[i].out_win[j].height; //输出高度


    config.comm_pool[k].blk_size = VICAP_ALIGN_UP((out_width * out_height * 3 / 2), VICAP_ALIGN_1K);//计算每个缓冲区的大小，并且进行对齐

    dev_obj[i].buf_size[j] = config.comm_pool[k].blk_size;//每个缓冲区大小
    printf("%s, dev(%d) chn(%d) pool(%d) buf_size(%d) blk_cnt(%d)\n", __func__, i, j, k ,dev_obj[i].buf_size[j], config.comm_pool[k].blk_cnt);
    
    ret = kd_mpi_vb_set_config(&config); //设置MPP视频缓存池属性
    if (ret) {
        printf("vb_set_config failed ret:%d\n", ret);
        return ret;
    }

    memset(&supplement_config, 0, sizeof(supplement_config));
    supplement_config.supplement_config |= VB_SUPPLEMENT_JPEG_MASK;

    ret = kd_mpi_vb_set_supplement_config(&supplement_config);//设置 VB 内存的附加信息，这里增加的是DCF信息，如拍摄的时间、拍摄的时候是否有闪光灯、数码缩放倍数等
    if (ret) {
        printf("vb_set_supplement_config failed ret:%d\n", ret);
        return ret;
    }

    ret = kd_mpi_vb_init();//初始化 MPP 视频缓存池
    if (ret) {
        printf("vb_init failed ret:%d\n", ret);
        return ret;
    }

    return 0;
}



#define VICAP_MIN_PARAMETERS (2)

static void usage(void)
{
    printf("usage: ./sample_vicap  -dev 0\n");
    printf("Options:\n");
    printf(" -dev:          vicap device id[0,1,2]\tdefault 0\n");

    printf(" -help:         print this help\n");

    exit(1);
}

extern k_u32 display_cnt;
extern k_u32 drop_cnt;
extern uint32_t hdr_buf_base_phy_addr;
extern void *hdr_buf_base_vir_addr;


static void vb_exit() {
    kd_mpi_vb_exit();
}

static uint64_t get_ticks()
{
    static volatile uint64_t time_elapsed = 0;
    __asm__ __volatile__(
        "rdtime %0"
        : "=r"(time_elapsed));
    return time_elapsed;
}

int main(int argc, char *argv[])
{
    // _set_mod_log(K_ID_VI, 6);
    k_s32 ret = 0;

    vicap_device_obj device_obj[VICAP_DEV_ID_MAX];
    memset(&device_obj, 0 , sizeof(device_obj));

    k_vicap_dev_attr dev_attr;
    k_vicap_chn_attr chn_attr;

    k_video_frame_info dump_info;

    k_u8 dev_count = 0, cur_dev = 0;
    k_u8 chn_count = 0, cur_chn = 0;

    k_u32 pipe_ctrl = 0xFFFFFFFF;
    memset(&dev_attr, 0, sizeof(k_vicap_dev_attr));

    if (argc < VICAP_MIN_PARAMETERS) {
        printf("sample_vicap requires some necessary parameters:\n");
        usage();
    }
    
    if (strcmp(argv[1], "-help") == 0)
    {
        usage();
    }
    else if (strcmp(argv[1], "-dev") == 0)
    {
        chn_count = 0;
        cur_dev =  atoi(argv[2]);;
        dev_count++;
        printf("cur_dev(%d), dev_count(%d)\n", cur_dev, dev_count);
        
        device_obj[cur_dev].dev_num = cur_dev; //sensor设备号
        device_obj[cur_dev].dev_enable = K_TRUE; //是否启用设备
        device_obj[cur_dev].ae_enable = K_TRUE;//default enable ae 自动曝光
        device_obj[cur_dev].awb_enable = K_TRUE;//default enable awb 自动白平衡
        device_obj[cur_dev].dnr3_enable = K_FALSE;//default disable 3ndr 3D降噪
        device_obj[cur_dev].hdr_enable = K_FALSE;//default disable hdr 高动态范围
        device_obj[cur_dev].dw_enable = K_FALSE;//default disable dw 宽动态模式
        
        //parse dev paramters
        printf("cur_chn(%d), chn_count(%d)\n", cur_chn ,chn_count); 
        device_obj[cur_dev].chn_num[cur_chn] = cur_chn; //VI通道的编号
        device_obj[cur_dev].chn_enable[cur_chn] = K_TRUE; //是否启动通道
        device_obj[cur_dev].preview[cur_chn] = K_TRUE;//default enable preview 通道的预览功能
        device_obj[cur_dev].sensor_type =  GC2093_MIPI_CSI2_1920X1080_30FPS_10BIT_LINEAR;//传感器类型
    }

    k_u16 out_width = 640;
    out_width = VICAP_ALIGN_UP(out_width, 16);
    device_obj[cur_dev].out_win[cur_chn ].width = out_width;


    k_u16 out_height = 640;
    // out_height = VICAP_ALIGN_UP(out_height, 16);
    device_obj[cur_dev].out_win[cur_chn].height = out_height;

    printf("sample_vicap: dev_count(%d), chn_count(%d)\n", dev_count, chn_count);

    int dev_num = 0;
    dev_attr.input_type = VICAP_INPUT_TYPE_SENSOR;//输入类型为sensor数据 
    //vicap get sensor info 根据指定的sensor配置类型获取sensor配置信息
    ret = kd_mpi_vicap_get_sensor_info(device_obj[dev_num].sensor_type, &device_obj[dev_num].sensor_info);
    if (ret) {
        printf("sample_vicap, the sensor type not supported!\n");
        return ret;
    }
    memcpy(&dev_attr.sensor_info, &device_obj[dev_num].sensor_info, sizeof(k_vicap_sensor_info));

    device_obj[dev_num].in_width = device_obj[dev_num].sensor_info.width; //1920
    device_obj[dev_num].in_height = device_obj[dev_num].sensor_info.height; //1080

    printf("sample_vicap, dev[%d] in size[%dx%d]\n", \
        dev_num, device_obj[dev_num].in_width, device_obj[dev_num].in_height);

    //vicap device attr set
    dev_attr.acq_win.h_start = 0; //采集窗口的水平起始位置
    dev_attr.acq_win.v_start = 0; //采集窗口的垂直起始位置
    dev_attr.acq_win.width = device_obj[dev_num].in_width; //采集窗口的宽度
    dev_attr.acq_win.height = device_obj[dev_num].in_height; //采集窗口的高度
    dev_attr.mode = VICAP_WORK_ONLINE_MODE; //设备工作模式为在线模式

    dev_attr.pipe_ctrl.data = pipe_ctrl; //设置ISP相关控制数据
    dev_attr.pipe_ctrl.bits.af_enable = 0; //禁用自动对焦
    dev_attr.pipe_ctrl.bits.ae_enable = device_obj[dev_num].ae_enable;
    dev_attr.pipe_ctrl.bits.awb_enable = device_obj[dev_num].awb_enable;
    dev_attr.pipe_ctrl.bits.dnr3_enable = device_obj[dev_num].dnr3_enable;
    dev_attr.pipe_ctrl.bits.ahdr_enable = device_obj[dev_num].hdr_enable;

    dev_attr.cpature_frame = 0; //捕获帧数
    dev_attr.dw_enable = device_obj[dev_num].dw_enable;

    dev_attr.mirror = device_obj[cur_dev].sensor_mirror; //设置镜像模式为设备对象中的传感器镜像模式

    ret = kd_mpi_vicap_set_dev_attr(dev_num, dev_attr);//设置VICAP设备属性
    if (ret) {
        printf("sample_vicap, kd_mpi_vicap_set_dev_attr failed.\n");
        return ret;
    }


    int chn_num = 0;
    //set default value 设置默认值
    if (!device_obj[dev_num].out_format[chn_num]) {
        device_obj[dev_num].out_format[chn_num] = PIXEL_FORMAT_YUV_SEMIPLANAR_420; //捕获图像格式为YUV420
    }

    
    if (!device_obj[dev_num].out_win[chn_num].width) {
        device_obj[dev_num].out_win[chn_num].width = device_obj[dev_num].in_width; //捕获图像宽度
    }

    if (!device_obj[dev_num].out_win[chn_num].height) {
        device_obj[dev_num].out_win[chn_num].height = device_obj[dev_num].in_height; //捕获图像高度
    }
    
    if ( device_obj[dev_num].out_win[chn_num].h_start || device_obj[dev_num].out_win[chn_num].v_start) {
        device_obj[dev_num].crop_enable[chn_num] = K_TRUE;
    }

    printf("sample_vicap, dev_num(%d), chn_num(%d), in_size[%dx%d], out_offset[%d:%d], out_size[%dx%d]\n", \
        dev_num, chn_num, device_obj[dev_num].in_width, device_obj[dev_num].in_height, \
        device_obj[dev_num].out_win[chn_num].h_start, device_obj[dev_num].out_win[chn_num].v_start, \
        device_obj[dev_num].out_win[chn_num].width, device_obj[dev_num].out_win[chn_num].height);

    ret = sample_vicap_vb_init(device_obj);//配置缓冲区
    if (ret) {
        printf("sample_vicap_vb_init failed\n");
        return -1;
    }
    atexit(vb_exit);//程序正常结束时，清理缓冲区

    //vicap channel attr set

    kd_mpi_vicap_set_dump_reserved(dev_num, chn_num, K_TRUE);//设置是否开启快速dump 模式

    memset(&chn_attr, 0, sizeof(k_vicap_chn_attr));

    chn_attr.out_win.width = device_obj[dev_num].out_win[chn_num].width; //设置输出图像宽度
    chn_attr.out_win.height = device_obj[dev_num].out_win[chn_num].height; //设置输出图像高度

    if (device_obj[dev_num].crop_enable[chn_num]) { //如果设置裁剪参数
        chn_attr.crop_win.width = device_obj[dev_num].crop_win[chn_num].width;  //chn_attr.out_win;1166;// 
        chn_attr.crop_win.height = device_obj[dev_num].crop_win[chn_num].height; //1944;//
        chn_attr.crop_win.h_start =device_obj[dev_num].out_win[chn_num].h_start;  //713;
        chn_attr.crop_win.v_start =device_obj[dev_num].out_win[chn_num].v_start;  //0;//
    } else {
        chn_attr.crop_win.width = device_obj[dev_num].in_width;
        chn_attr.crop_win.height = device_obj[dev_num].in_height;
    }

    chn_attr.scale_win = chn_attr.out_win;
    chn_attr.crop_enable = device_obj[dev_num].crop_enable[chn_num];
    chn_attr.scale_enable = K_FALSE; //缩放关闭
    chn_attr.chn_enable = K_TRUE; //通道使能

    chn_attr.pix_format = device_obj[dev_num].out_format[chn_num];//图像格式
    chn_attr.buffer_num = VICAP_OUTPUT_BUF_NUM;//缓冲区数量
    chn_attr.buffer_size = device_obj[dev_num].buf_size[chn_num];//缓冲区大小

    printf("sample_vicap, set dev(%d) chn(%d) attr, buffer_size(%d), out size[%dx%d]\n", \
        dev_num, chn_num, chn_attr.buffer_size, chn_attr.out_win.width, chn_attr.out_win.height);

    printf("sample_vicap out_win h_start is %d ,v_start is %d \n", chn_attr.out_win.h_start, chn_attr.out_win.v_start);

    ret = kd_mpi_vicap_set_chn_attr(dev_num, chn_num, chn_attr);//设置VICAP设备通道属性
    if (ret) {
        printf("sample_vicap, kd_mpi_vicap_set_chn_attr failed.\n");
        goto vb_exit;
    }

    printf("sample_vicap, vicap dev(%d) init\n", dev_num);
    ret = kd_mpi_vicap_init(dev_num);//VICAP设备初始化
    if (ret) {
        printf("sample_vicap, vicap dev(%d) init failed.\n", dev_num);
        goto app_exit;
    }



    printf("sample_vicap, vicap dev(%d) start stream\n", dev_num);
    ret = kd_mpi_vicap_start_stream(dev_num);//启动VICAP设备输出数据流
    if (ret) {
        printf("sample_vicap, vicap dev(%d) start stream failed.\n", dev_num);
        goto app_exit;
    }


    static k_u32 dump_count = 0;

    k_char select = 0;
    while(K_TRUE)
    {
        if(select != '\n')
        {
            printf("---------------------------------------\n");
            printf(" Input character to select test option\n");
            printf("---------------------------------------\n");
            printf(" d: dump data addr test\n");
            printf(" q: to exit\n");
            printf("---------------------------------------\n");
            printf("please Input:\n\n");
        }
        select = (k_char)getchar();
        switch (select)
        {
        case 'd': //获设备中转储（dump）一帧图像数据，并将其保存到文件中
            printf("sample_vicap... dump frame.\n");
            int dev_num = 0;
            int chn_num = 0;
            printf("sample_vicap, dev(%d) chn(%d) dump frame.\n", dev_num, chn_num);

            memset(&dump_info, 0 , sizeof(k_video_frame_info));
            uint64_t start = get_ticks(); //获取当前时间戳
            ret = kd_mpi_vicap_dump_frame(dev_num, chn_num, VICAP_DUMP_YUV, &dump_info, 1000);//根据指定的设备和输出通道dump vicap数据
            if (ret) {
                printf("sample_vicap, dev(%d) chn(%d) dump frame failed.\n", dev_num, chn_num);
                continue;
            }
            uint64_t end = get_ticks();//获取结束时间
            printf("dump cost %lu us\n", (end - start) / 27);

            k_char *suffix;
            k_u32 data_size = 0;
            k_u8 lbit = 0;
            k_u8 *virt_addr = NULL;
            k_char filename[256];

            if (dump_info.v_frame.pixel_format == PIXEL_FORMAT_YUV_SEMIPLANAR_420) {
                suffix = "yuv420sp";
                data_size = dump_info.v_frame.width * dump_info.v_frame.height * 3 /2;//计算一帧图像的大小
            }else {
                suffix = "unkown";
            }

            virt_addr = kd_mpi_sys_mmap(dump_info.v_frame.phys_addr[0], data_size);//memory 存储映射接口，将物理地址映射到虚拟地址
            if (virt_addr) {
                memset(filename, 0 , sizeof(filename));

                snprintf(filename, sizeof(filename), "dev_%02d_chn_%02d_%dx%d_%04d.%s", \
                    dev_num, chn_num, dump_info.v_frame.width, dump_info.v_frame.height, dump_count, suffix);//生成文件名

                printf("save dump data to file(%s)\n", filename);
                FILE *file = fopen(filename, "wb+");
                if (file) {
                    if (device_obj[dev_num].dalign && lbit) {
                        for (k_u32 index = 0; index < data_size; index += 2) {
                            k_u16 raw_data = (virt_addr[index + 1] << 8 ) | virt_addr[index];
                            raw_data = raw_data << lbit;
                            fwrite(&raw_data, sizeof(raw_data), 1, file);//保存文件
                        }
                    } else {
                        fwrite(virt_addr, 1, data_size, file);
                    }
                    fclose(file);//关闭文件
                } else {
                    printf("sample_vicap, open dump file failed(%s)\n", strerror(errno));
                }

                kd_mpi_sys_munmap(virt_addr, data_size);//解除映射
            } else {
                printf("sample_vicap, map dump addr failed.\n");
            }

            printf("sample_vicap, release dev(%d) chn(%d) dump frame.\n", dev_num, chn_num);

            ret = kd_mpi_vicap_dump_release(dev_num, chn_num, &dump_info);//释放dump数据帧
            if (ret) {
                printf("sample_vicap, dev(%d) chn(%d) release dump frame failed.\n", dev_num, chn_num);
            }
            dump_count++;
            break;
        case 'q':
            goto app_exit;
        default:
            break;
        }
        sleep(1);
    }

app_exit:

    for (int dev_num = 0; dev_num < VICAP_DEV_ID_MAX; dev_num++) {
        if (!device_obj[dev_num].dev_enable)
            continue;

        printf("sample_vicap, vicap dev(%d) stop stream\n", dev_num);
        ret = kd_mpi_vicap_stop_stream(dev_num);
        if (ret) {
            printf("sample_vicap, vicap dev(%d) stop stream failed.\n", dev_num);
        }
        printf("display_cnt[%d], drop_cnt[%d]\n", display_cnt, drop_cnt);

        printf("sample_vicap, vicap dev(%d) deinit\n", dev_num);
        ret = kd_mpi_vicap_deinit(dev_num);
        if (ret) {
            printf("sample_vicap, vicap dev(%d) deinit failed.\n", dev_num);
        }

    }
    for (int dev_num = 0; dev_num < VICAP_DEV_ID_MAX; dev_num++) {
        if (!device_obj[dev_num].dev_enable)
            continue;

        for (int chn_num = 0; chn_num < VICAP_CHN_ID_MAX; chn_num++) {
            if (!device_obj[dev_num].chn_enable[chn_num])
                continue;
        }
    }


    printf("Press Enter to exit!!!!\n");
    getchar();

    /*Allow one frame time for the VO to release the VB block*/
    k_u32 display_ms = 1000 / 33;
    usleep(1000 * display_ms);

vb_exit:

    return ret;
}
