include $(MPP_SRC_DIR)/middleware/mpp.mk
include $(MPP_SRC_DIR)/middleware/rt-smart.mk

CURRECT_DIR_NAME=$(shell basename `pwd`)
LOCAL_SRC_DIR = $(shell pwd)
BIN = $(MPP_SRC_DIR)/userapps/sample/elf/$(CURRECT_DIR_NAME).elf
LIBPATH = $(MPP_LIB_PATH)
LIBS = $(MPP_LIBS)

LIBS_FOR_CONSOLE_APPLICATION = -lssl -lcrypto
CC_CFLAGS_EX=-mcmodel=medany -march=rv64imafdcv -mabi=lp64d -Wall -O0 -g -gdwarf-2 -n --static $(KCFLAGS)

FFMPEG_DIR = $(MPP_SRC_DIR)/middleware/src/ffmpeg/src/lib_ffmpeg
X264_DIR = $(MPP_SRC_DIR)/middleware/src/x264/src/lib_x264
KDMEDIA = $(MPP_SRC_DIR)/middleware/src/kdmedia
RTSP_PUSHER = $(MPP_SRC_DIR)/middleware/src/rtsp_pusher

LOCAL_LIBS_PATH = -L$(KDMEDIA) -L$(FFMPEG_DIR)/lib -L$(X264_DIR)/lib -L$(RTSP_PUSHER)
LOCAL_LIBS = -lkdmedia -lrtsp_pusher -lavformat -lavcodec  -lavutil -lswresample -lpostproc -lx264 -lpthread

LOCAL_CFLAGS = -I$(LOCAL_SRC_DIR) -I$(FFMPEG_DIR)/include -I$(KDMEDIA)/include -I$(RTSP_PUSHER)/include

WERROR_FLAGS = -Wno-register -Wno-multichar -Wno-pessimizing-move -Wno-deprecated-declarations -Wno-unused-result -Wno-unused-variable -Wno-format -Wno-return-type -Wno-sign-compare -Wno-unused-label

CXX_FLAGS = $(WERROR_FLAGS)

SRCS = $(wildcard $(LOCAL_SRC_DIR)/*.c)

OBJS = $(patsubst %.c,%.o,$(SRCS))

CPP_SRCS = $(wildcard $(LOCAL_SRC_DIR)/*.cpp)

CPP_OBJS = $(patsubst %.cpp,%.o,$(CPP_SRCS))

all:$(BIN)
	#@-rm -f *.o
	@echo "${PWD}/Makefile all"

$(OBJS): %.o : %.c
	@echo CC $@
	@echo $(LOCAL_CFLAGS)
	@$(CC) $(CC_CFLAGS_EX) $(LOCAL_CFLAGS) $(BSP_CFLGAS) $(RTSMART_CFLAGS) $(MPP_USER_CFLGAS) $(LOCAL_LIBS) -c $< -o $@

$(CPP_OBJS): %.o : %.cpp
	@echo CXX $@
	@echo $(LOCAL_CFLAGS)
	@$(CPP) $(CC_CFLAGS_EX) $(CXX_FLAGS) $(LOCAL_CFLAGS) $(BSP_CFLGAS) $(RTSMART_CFLAGS) $(MPP_USER_CFLGAS) $(LOCAL_LIBS) -c $< -o $@

$(BIN): $(OBJS) $(CPP_OBJS)
	@echo LD $@
	@$(CPP) -o $(BIN) $(LINKFLAG) -Wl,--whole-archive -Wl,--no-whole-archive -n --static $^ -L$(LIBPATH) $(CXX_FLAGS) $(LOCAL_LIBS) $(LOCAL_LIBS_PATH) -Wl,--start-group $(LIBS) -Wl,--end-group

clean:
	echo "${PWD}/Makefile clean"
	rm -rf $(BIN) $(OBJS) $(CPP_OBJS)

.PHONY: all clean
