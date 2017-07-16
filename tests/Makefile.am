bin_PROGRAMS = yamidecode yamiencode yamivpp yamitranscode
if ENABLE_V4L2
bin_PROGRAMS += v4l2encode v4l2decode
endif

#to compile within yocto
extra_includes = \
        -I$(top_srcdir) \
        $(NULL)

AM_CFLAGS = \
	$(LIBVA_CFLAGS) \
	$(LIBYAMI_CFLAGS) \
	$(extra_includes) \
	$(NULL)

DECODE_INPUT_SOURCES = \
	decodeinput.cpp \
	$(NULL)

YAMI_COMMON_LIBS = \
	$(LIBVA_LIBS) \
	$(LIBVA_DRM_LIBS) \
	$(LIBYAMI_LIBS) \
	$(NULL)

YAMI_DECODE_LIBS = \
	$(YAMI_COMMON_LIBS) \
	$(NULL)

if ENABLE_V4L2
AM_CFLAGS += \
	$(LIBYAMIV4L2_CFLAGS) \
	$(NULL)

YAMI_COMMON_LIBS += \
	$(LIBYAMIV4L2_LIBS) \
	$(NULL)
endif

if ENABLE_AVFORMAT
DECODE_INPUT_SOURCES += decodeinputavformat.cpp
AM_CFLAGS += -D__STDC_CONSTANT_MACROS \
	$(LIBAVFORMAT_CFLAGS)
YAMI_DECODE_LIBS += $(LIBAVFORMAT_LIBS)
endif

AM_CPPFLAGS = $(AM_CFLAGS)

if ENABLE_X11
YAMI_COMMON_LIBS += $(LIBVA_X11_LIBS) -lX11
endif

if ENABLE_WAYLAND
YAMI_COMMON_LIBS += $(LIBVA_WAYLAND_LIBS)
endif
if ENABLE_TESTS_GLES
YAMI_DECODE_LIBS += $(LIBEGL_LIBS) $(LIBGLES2_LIBS)
endif

if ENABLE_MD5
YAMI_DECODE_LIBS += $(LIBBSD_LIBS)
endif

YAMI_ENCODE_LIBS = \
	$(YAMI_DECODE_LIBS) \
	$(NULL)

V4L2_DECODE_LIBS = \
	$(YAMI_DECODE_LIBS) \
	$(NULL)

V4L2_DECODE_LIBS += $(LIBEGL_LIBS) $(LIBGLES2_LIBS)

V4L2_ENCODE_LIBS = \
	$(YAMI_ENCODE_LIBS) \
	$(NULL)

CAPI_DECODE_LIBS = \
	$(YAMI_DECODE_LIBS) \
	$(NULL)

CAPI_ENCODE_LIBS = \
	$(YAMI_ENCODE_LIBS) \
	$(NULL)

YAMI_VPP_LIBS = \
	$(YAMI_DECODE_LIBS) \
	$(YAMI_ENCODE_LIBS) \
	$(NULL)

YAMI_VPP_CFLAGS = \
	$(LIBVA_CFLAGS) \
	$(NULL)

yamidecode_LDADD    = $(YAMI_VPP_LIBS)
yamidecode_LDFLAGS  = $(YAMI_VPP_LDFLAGS)
yamidecode_SOURCES  = decode.cpp decodehelp.cpp $(DECODE_INPUT_SOURCES) decodeoutput.cpp vppinputoutput.cpp vppinputdecode.cpp vppoutputencode.cpp encodeinput.cpp encodeInputCamera.cpp encodeInputDecoder.cpp vppinputdecodecapi.cpp
if ENABLE_TESTS_GLES
yamidecode_SOURCES += ../egl/egl_util.c ./egl/gles2_help.c
endif

yamiencode_LDADD    = $(YAMI_ENCODE_LIBS)
yamiencode_LDFLAGS  = $(YAMI_ENCODE_LDFLAGS)
yamiencode_SOURCES  = encode.cpp encodeinput.cpp encodeInputCamera.cpp encodeInputDecoder.cpp $(DECODE_INPUT_SOURCES)

v4l2decode_LDADD   = $(V4L2_DECODE_LIBS)
v4l2decode_LDFLAGS = $(V4L2_DECODE_LDFLAGS)
v4l2decode_SOURCES = v4l2decode.cpp V4L2Renderer.cpp V4L2Device.cpp decodehelp.cpp $(DECODE_INPUT_SOURCES)

if ENABLE_TESTS_GLES
v4l2decode_SOURCES += ./egl/gles2_help.c
endif
v4l2decode_LDADD += -ldl


v4l2encode_LDADD   = $(V4L2_ENCODE_LIBS)
v4l2encode_LDFLAGS = $(V4L2_ENCODE_LDFLAGS)
v4l2encode_SOURCES = v4l2encode.cpp encodeinput.h encodeinput.cpp encodeInputCamera.cpp encodeInputDecoder.cpp $(DECODE_INPUT_SOURCES)

yamivpp_LDADD    = $(YAMI_VPP_LIBS)
yamivpp_LDFLAGS  = $(YAMI_VPP_LDFLAGS)
yamivpp_SOURCES  = vppinputdecode.cpp vppinputoutput.cpp vppoutputencode.cpp  vpp.cpp encodeinput.cpp encodeInputCamera.cpp encodeInputDecoder.cpp $(DECODE_INPUT_SOURCES) vppinputdecodecapi.cpp

LIBXCAM_HOME_DIR = /home/ycyang/Desktop/VE450/libxcam
LIBYAMI_HOME_DIR = /home/ycyang/Desktop/VE450/libyami

OPENCV_FLAGS = -I/usr/local/include/opencv -I/usr/local/include  

XCAM_LIBDRM_CFLAGS = -I/usr/include/libdrm
XCAM_LIBDRM_LIBS = -ldrm

XCORE_LA = $(LIBXCAM_HOME_DIR)/xcore/libxcam_core.la
XCORE_DIR = $(LIBXCAM_HOME_DIR)/xcore
OCL_LA =  $(LIBXCAM_HOME_DIR)/modules/ocl/libxcam_ocl.la
OCL_DIR = $(LIBXCAM_HOME_DIR)/modules/ocl

XCAM_CXXFLAGS = -fPIC -DSTDC99 -W -D_REENTRANT -Wformat -Wformat-security -fstack-protector -std=c++0x

XCAM_FLAG_3 = $(XCAM_LIBDRM_CFLAGS) $(XCAM_LIBDRM_LIBS)
XCAM_FLAG_4 = -I$(LIBXCAM_HOME_DIR)/ext/atomisp $(NULL)

xcam_tests_cxxflags = $(XCAM_CXXFLAGS) $(XCAM_FLAG_3) $(XCAM_FLAG_4)
CAPI_LIBS = $(LIBXCAM_HOME_DIR)/capi/libxcam_capi.la
XCAM_LIBS = $(XCORE_LA) $(OCL_LA)
VPP_LIBS = $(LIBYAMI_HOME_DIR)/vpp/libyami_vpp.la
VAAPI_LIBS = $(LIBYAMI_HOME_DIR)/vaapi/libyami_vaapi.la
CAPI_DIR = $(LIBXCAM_HOME_DIR)/capi
XCAM_DIR = -I$(XCORE_DIR) -I$(OCL_DIR)
VPP_DIR = $(LIBYAMI_HOME_DIR)/vpp
VAAPI_DIR = $(LIBYAMI_HOME_DIR)/vaapi


yamitranscode_LDADD    = $(YAMI_VPP_LIBS) $(VPP_LIBS) $(VAAPI_LIBS) $(XCAM_LIBS) $(CAPI_LIBS) $(LIBYAMI_HOME_DIR)/common/libyami_common.la
yamitranscode_LDFLAGS  = -pthread $(YAMI_VPP_LDFLAGS)
yamitranscode_SOURCES  = vppinputdecode.cpp vppinputoutput.cpp vppoutputencode.cpp  yamitranscode.cpp encodeinput.cpp encodeInputCamera.cpp encodeInputDecoder.cpp $(DECODE_INPUT_SOURCES) vppinputasync.cpp vppinputdecodecapi.cpp
yamitranscode_CXXFLAGS = -I$(VPP_DIR) $(XCAM_DIR) $(NULL) $(OPENCV_FLAGS) $(xcam_tests_cxxflags) -I$(CAPI_DIR) -I$(VAAPI_DIR) -I$(LIBYAMI_HOME_DIR)


bin_PROGRAMS += yamiinfo
yamiinfo_SOURCES = yamiinfo.cpp
yamiinfo_LDADD = $(YAMI_COMMON_LIBS)
yamiinfo_LDFLAGS = -Wl,--no-as-needed \
	$(LIBYAMI_CFLAGS) \
	$(NULL)
