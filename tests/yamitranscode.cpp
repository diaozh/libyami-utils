/*
 * Copyright (C) 2011-2014 Intel Corporation. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#define XCAM_VERSION 0x100

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "common/PooledFrameAllocator.h"
#include "vppinputdecode.h"
#include "vppinputoutput.h"
#include "vppoutputencode.h"
#include "encodeinput.h"
#include "tests/vppinputasync.h"
#include "common/log.h"
#include <Yami.h>
#include <stdio.h>
#include <stdlib.h>
#include <getopt.h>

#include <va/va_drmcommon.h>
#include "vaapipostprocess_base.h"
//#include "capi/context_priv.h"
#include <unistd.h>
#include "cl_device.h"
#include "cl_context.h"
#include "drm_display.h"
#include "image_file_handle.h"
#include "dma_video_buffer.h"
#include "cl_fisheye_handler.h"
#include "cl_image_360_stitch.h"
#include "capi/xcam_handle.h"
#include "capi/context_priv.h"
#include "vaapidisplay.h"

#include <iostream>
#include <string>

using namespace YamiMediaCodec;
using namespace XCam;


static void print_help(const char* app)
{
    printf("%s <options>\n", app);
    printf("   -i <source filename> load a raw yuv file or a compressed video file\n");
    printf("   -W <width> -H <height>\n");
    printf("   -o <coded file> optional\n");
    printf("   -b <bitrate: kbps> optional\n");
    printf("   -f <frame rate> optional\n");
    printf("   -c <codec: HEVC|AVC|VP8|JPEG>\n");
    printf("   -s <fourcc: I420|NV12|IYUV|YV12>\n");
    printf("   -N <number of frames to encode(camera default 50), useful for camera>\n");
    printf("   -t <AVC scalability temporal layer number  (default 1)> optional\n");
    printf("   --qp <initial qp> optional\n");
    printf("   --rcmode <CBR|CQP|VBR> optional\n");
    printf("   --target-percnetage <target percentage of bitrate in VBR mode, default 95, range in(50-100)> optional\n");
    printf("   --hrd-window-size <windows size in milliseconds, default 1000> optional\n");
    printf("   --vbv-buffer-fullness <vbv initial buffer fullness in bit> optional\n");
    printf("   --vbv-buffer-size <vbv buffer size in bit> optional\n");
    printf("   --ipperiod <0 (I frame only) | 1 (I and P frames) | N (I,P and B frames, B frame number is N-1)> optional\n");
    printf("   --intraperiod <Intra frame period(default 30)> optional\n");
    printf("   --refnum <number of referece frames(default 1)> optional\n");
    printf("   --idrinterval <AVC/HEVC IDR frame interval(default 0)> optional\n");
    printf("   --disable-cabac <AVC is to use CABAC or not, for Main and High Profile, default is enabled\n");
    printf("   --enable-dct8x8 <AVC is to use DCT8x8 or not, for High Profile, default is disabled\n");
    printf("   --disable-deblock <AVC is to use Deblock or not, default is enabled\n");
    printf("   --deblockalphadiv2 <AVC Alpha offset of debloking filter divided 2 (default 2)> optional\n");
    printf("   --deblockbetadiv2 <AVC Beta offset of debloking filter divided 2 (default 2)> optional\n");
    printf("   --qpip <qp difference between adjacent I/P (default 0)> optional\n");
    printf("   --qpib <qp difference between adjacent I/B (default 0)> optional\n");
    printf("   --priorityid <AVC priority_id of prefix nal unit (default 0)> optional\n");
    printf("   --ow <output width> optional\n");
    printf("   --oh <output height> optional\n");
    printf("   --btl0 <svc-t layer 0 bitrate: kbps> optional\n");
    printf("   --btl1 <svc-t layer 1 bitrate: kbps > optional\n");
    printf("   --btl2 <svc-t layer 2 bitrate: kbps> optional\n");
    printf("   --btl3 <svc-t layer 3 bitrate: kbps> optional\n");
    printf("   --lowpower <Enable AVC low power mode (default 0, Disabled)> optional\n");
    printf("   --quality-level <encoded video qulity level(default 0), range[%d, %d]> optional\n",
        VIDEO_PARAMS_QUALITYLEVEL_NONE, VIDEO_PARAMS_QUALITYLEVEL_MAX);
    printf("   VP9 encoder specific options:\n");
    printf("   --refmode <VP9 Reference frames mode (default 0 last(previous), "
           "gold/alt (previous key frame) | 1 last (previous) gold (one before "
           "last) alt (one before gold)> optional\n");
}

static VideoRateControl string_to_rc_mode(char *str)
{
    VideoRateControl rcMode;

    if (!strcasecmp (str, "CBR"))
        rcMode = RATE_CONTROL_CBR;
    else if (!strcasecmp(str, "VBR")) {
        rcMode = RATE_CONTROL_VBR;
    }
    else if (!strcasecmp (str, "CQP"))
        rcMode = RATE_CONTROL_CQP;
    else {
        printf("Unsupport  RC mode\n");
        rcMode = RATE_CONTROL_NONE;
    }
    return rcMode;
}

static bool processCmdLine(int argc, char *argv[], TranscodeParams& para)
{
    char opt;
    const struct option long_opts[] = {
        { "help", no_argument, NULL, 'h' },
        { "qp", required_argument, NULL, 0 },
        { "rcmode", required_argument, NULL, 0 },
        { "ipperiod", required_argument, NULL, 0 },
        { "intraperiod", required_argument, NULL, 0 },
        { "refnum", required_argument, NULL, 0 },
        { "idrinterval", required_argument, NULL, 0 },
        { "disable-cabac", no_argument, NULL, 0 },
        { "enable-dct8x8", no_argument, NULL, 0 },
        { "disable-deblock", no_argument, NULL, 0 },
        { "deblockalphadiv2", required_argument, NULL, 0 },
        { "deblockbetadiv2", required_argument, NULL, 0 },
        { "qpip", required_argument, NULL, 0 },
        { "qpib", required_argument, NULL, 0 },
        { "priorityid", required_argument, NULL, 0 },
        { "refmode", required_argument, NULL, 0 },
        { "ow", required_argument, NULL, 0 },
        { "oh", required_argument, NULL, 0 },
        { "btl0", required_argument, NULL, 0 },
        { "btl1", required_argument, NULL, 0 },
        { "btl2", required_argument, NULL, 0 },
        { "btl3", required_argument, NULL, 0 },
        { "lowpower", no_argument, NULL, 0 },
        { "target-percnetage", required_argument, NULL, 0 },
        { "hrd-window-size", required_argument, NULL, 0 },
        { "vbv-buffer-fullness", required_argument, NULL, 0 },
        { "vbv-buffer-size", required_argument, NULL, 0 },
        { "quality-level", required_argument, NULL, 0 },
        { NULL, no_argument, NULL, 0 }
    };
    int option_index;

    if (argc < 2) {
        fprintf(stderr, "can not encode without option, please type 'yamitranscode -h' to help\n");
        return false;
    }

    while ((opt = getopt_long_only(argc, argv, "W:H:b:f:c:s:i:o:N:h:t:", long_opts,&option_index)) != -1)
    {
        switch (opt) {
        case 'h':
        case '?':
            print_help (argv[0]);
            return false;
        case 'i':
            para.inputFileName = optarg;
            break;
        case 'o':
            para.outputFileName = optarg;
            break;
        case 'W':
            para.iWidth = atoi(optarg);
            break;
        case 'H':
            para.iHeight = atoi(optarg);
            break;
        case 'b':
            para.m_encParams.bitRate = atoi(optarg) * 1024;//kbps to bps
            break;
        case 'f':
            para.m_encParams.fps = atoi(optarg);
            break;
        case 'c':
            para.m_encParams.codec = optarg;
            break;
        case 's':
            if (strlen(optarg) == 4)
                para.fourcc = VA_FOURCC(optarg[0], optarg[1], optarg[2], optarg[3]);
            break;
        case 'N':
            para.frameCount = atoi(optarg);
            break;
        case 't':
            para.m_encParams.temporalLayerNum = atoi(optarg);
            break;
        case 0:
             switch (option_index) {
                case 1:
                    para.m_encParams.initQp = atoi(optarg);
                    break;
                case 2:
                    para.m_encParams.rcMode = string_to_rc_mode(optarg);
                    break;
                case 3:
                    para.m_encParams.ipPeriod = atoi(optarg);
                    break;
                case 4:
                    para.m_encParams.intraPeriod = atoi(optarg);
                    break;
                case 5:
                    para.m_encParams.numRefFrames= atoi(optarg);
                    break;
                case 6:
                    para.m_encParams.idrInterval = atoi(optarg);
                    break;
                case 7:
                    para.m_encParams.enableCabac = false;
                    break;
                case 8:
                    para.m_encParams.enableDct8x8 = true;
                    break;
                case 9:
                    para.m_encParams.enableDeblockFilter = false;
                    break;
                case 10:
                    para.m_encParams.deblockAlphaOffsetDiv2 = atoi(optarg);
                    break;
                case 11:
                    para.m_encParams.deblockBetaOffsetDiv2 = atoi(optarg);
                    break;
                case 12:
                    para.m_encParams.diffQPIP = atoi(optarg);
                    break;
                case 13:
                    para.m_encParams.diffQPIB = atoi(optarg);
                    break;
                case 14:
                    para.m_encParams.priorityId = atoi(optarg);
                    break;
                case 15:
                    para.m_encParams.m_encParamsVP9.referenceMode = atoi(optarg);
                    break;
                case 16:
                    para.oWidth = atoi(optarg);
                    break;
                case 17:
                    para.oHeight = atoi(optarg);
                    break;
                case 18:
                    para.m_encParams.layerBitRate[0] = atoi(optarg) * 1024;//kbps to bps;
                    break;
                case 19:
                    para.m_encParams.layerBitRate[1] = atoi(optarg) * 1024;//kbps to bps;
                    break;
                case 20:
                    para.m_encParams.layerBitRate[2] = atoi(optarg) * 1024;//kbps to bps;
                    break;
                case 21:
                    para.m_encParams.layerBitRate[3] = atoi(optarg) * 1024;//kbps to bps;
                    break;
                case 22:
                    para.m_encParams.enableLowPower = true;
                    break;
                case 23:
                    para.m_encParams.targetPercentage = atoi(optarg);
                    break;
                case 24:
                    para.m_encParams.windowSize = atoi(optarg);
                    break;
                case 25:
                    para.m_encParams.initBufferFullness = atoi(optarg);
                    break;
                case 26:
                    para.m_encParams.bufferSize = atoi(optarg);
                    break;
                case 27:
                    para.m_encParams.qualityLevel = atoi(optarg);
                    break;
            }
        }
    }
    if (optind < argc) {
        int indexOpt = optind;
        printf("unrecognized option: ");
        while (indexOpt < argc)
            printf("%s ", argv[indexOpt++]);
        printf("\n");
        print_help(argv[0]);
        return false;
    }

    if (para.inputFileName.empty()) {
        fprintf(stderr, "can not encode without input file\n");
        return false;
    }
    if (para.outputFileName.empty())
        para.outputFileName = "test.264";

    if ((para.m_encParams.rcMode == RATE_CONTROL_CBR) && (para.m_encParams.bitRate <= 0)) {
        fprintf(stderr, "please make sure bitrate is positive when CBR mode\n");
        return false;
    }

    if ((para.m_encParams.rcMode == RATE_CONTROL_VBR) && (para.m_encParams.bitRate <= 0)) {
        fprintf(stderr, "please make sure bitrate is positive when VBR mode\n");
        return false;
    }

    if (!strncmp(para.inputFileName.c_str(), "/dev/video", strlen("/dev/video")) && !para.frameCount)
        para.frameCount = 50;

    if (!para.oWidth)
        para.oWidth = para.iWidth;
    if (!para.oHeight)
        para.oHeight = para.iHeight;

    return true;
}

SharedPtr<VppInput> createInput(TranscodeParams& para, const SharedPtr<VADisplay>& display)
{
    SharedPtr<VppInput> input(VppInput::create(para.inputFileName.c_str(), para.fourcc, para.iWidth, para.iHeight));
    if (!input) {
        ERROR("creat input failed");
        return input;
    }
    SharedPtr<VppInputFile> inputFile = DynamicPointerCast<VppInputFile>(input);
    if (inputFile) {
        SharedPtr<FrameReader> reader(new VaapiFrameReader(display));
        SharedPtr<FrameAllocator> alloctor(new YamiMediaCodec::PooledFrameAllocator(display, 5));
        if(!inputFile->config(alloctor, reader)) {
            ERROR("config input failed");
            input.reset();
        }
    }
    SharedPtr<VppInputDecode> inputDecode = DynamicPointerCast<VppInputDecode>(input);
    if (inputDecode) {
        NativeDisplay nativeDisplay;
        nativeDisplay.type = NATIVE_DISPLAY_VA;
        nativeDisplay.handle = (intptr_t)*display;
        if(!inputDecode->config(nativeDisplay)) {
            ERROR("config input decode failed");
            input.reset();
        }
    }
    if (input)
        input = VppInputAsync::create(input, 3); //make input in other thread.
    return input;
}

SharedPtr<VppOutput> createOutput(TranscodeParams& para, const SharedPtr<VADisplay>& display, uint32_t fourcc)
{
    SharedPtr<VppOutput> output = VppOutput::create(
        para.outputFileName.c_str(), fourcc, para.oWidth, para.oHeight,
        para.m_encParams.codec.c_str(), para.m_encParams.fps);
    SharedPtr<VppOutputFile> outputFile = DynamicPointerCast<VppOutputFile>(output);
    if (outputFile) {
        SharedPtr<FrameWriter> writer(new VaapiFrameWriter(display));
        if (!outputFile->config(writer)) {
            ERROR("config writer failed");
            output.reset();
        }
        return output;
    }
    SharedPtr<VppOutputEncode> outputEncode = DynamicPointerCast<VppOutputEncode>(output);
    if (outputEncode) {
        NativeDisplay nativeDisplay;
        nativeDisplay.type = NATIVE_DISPLAY_VA;
        nativeDisplay.handle = (intptr_t)*display;
        if (!outputEncode->config(nativeDisplay, &para.m_encParams)) {
            ERROR("config output encode failed");
            output.reset();
        }
        return output;
    }
    return output;
}

SharedPtr<FrameAllocator> createAllocator(const SharedPtr<VppOutput>& output, const SharedPtr<VADisplay>& display, int32_t extraSize)
{
    uint32_t fourcc;
    int width, height;
    SharedPtr<FrameAllocator> allocator(new YamiMediaCodec::PooledFrameAllocator(display, std::max(extraSize, 5)));
    if (!output->getFormat(fourcc, width, height)
        || !allocator->setFormat(fourcc, width,height)) {
        allocator.reset();
        ERROR("get Format failed");
    }
    return allocator;
}

class StitchPostProcess : public VaapiPostProcessBase {
    int flag = 0;
    XCamHandle * stitch_handle;
    virtual YamiStatus process(const SharedPtr<VideoFrame>& src,
                               const SharedPtr<VideoFrame>& dest)
	{
        if (flag++ == 0)
        {
            stitch_handle = xcam_create_handle ("Stitch");
            printf("Setting paramters!\n");
            xcam_handle_set_parameters (stitch_handle, "width" , "1920", "height", "1080", NULL);

            printf("Initializing handle!\n");
            xcam_handle_init(stitch_handle);
        }
		printf("Start of Process!\n");
		printf("Get src surface!\n");
		VASurfaceID surface = (VASurfaceID)src->surface;
		printf("src surface:\t%ld\n", src->surface);
		VAImage image;
		
		printf("Get src Image!\n");
		
		VAStatus status = vaDeriveImage(m_display->getID(), surface, &image);
		// TODO check here
		printf("VaDeriveImage status: %d\n",status);
		printf("Description: %s\n",vaErrorStr(status));
		printf("Get src bufferinfo!\n");
		VABufferInfo m_bufferInfo;
		m_bufferInfo.mem_type = VA_SURFACE_ATTRIB_MEM_TYPE_DRM_PRIME;
		status = vaAcquireBufferHandle(m_display->getID(), image.buf, &m_bufferInfo);
		printf("src fourcc:\t%s\n",&src->fourcc);
		printf("src width:\t%u\t(Crop.width)\n",src->crop.width);
		printf("src height:\t%u\t(Crop.height)\n",src->crop.height);
		printf("src x:\t%u\t(Crop.x)\n",src->crop.x);
		printf("src y:\t%u\t(Crop.y)\n",src->crop.y);
		printf("src width:\t%u\t(Image.width)\n",image.width);
		printf("src height:\t%u\t(Image.height)\n",image.height);
		printf("pitches:\t%u\t(Pitches[0])\n", image.pitches[0]);
		printf("offset: \t%u\t(Offsets[1])\n",image.offsets[1]);
		printf("ali weight:\t%u\n",image.pitches[0]);
		printf("ali height:\t%u\n",image.offsets[1]/image.pitches[0]);		
		printf("handle: \t%lu\n",m_bufferInfo.handle);
		printf("Set src dma buf!\n");
		
		DmaVideoBuffer *src_dma_buf;
		VideoBufferInfo src_buf_info;
		//VideoFrame * temp = src.get();

		src_buf_info.init(src->fourcc, src->crop.width, src->crop.height, 1920, 1088);
		src_dma_buf = new DmaVideoBuffer(src_buf_info, m_bufferInfo.handle);
		src_dma_buf->set_timestamp (src->timeStamp);
		printf("Get dst surface!\n");
		VASurfaceID dest_surface = (VASurfaceID)dest->surface;
		printf("dst surface:\t%ld\n", dest->surface);
		VAImage dest_image;
		printf("Get dst image!\n");

		status = vaDeriveImage(m_display->getID(), dest_surface, &dest_image);
		
		printf("VaDeriveImage status: %d\n",status);
		printf("Description: %s\n",vaErrorStr(status));
		// TODO check here
		printf("Get dst bufferinfo!\n");
		VABufferInfo dest_m_bufferInfo;
		dest_m_bufferInfo.mem_type = VA_SURFACE_ATTRIB_MEM_TYPE_DRM_PRIME;
		status = vaAcquireBufferHandle(m_display->getID(), dest_image.buf, &dest_m_bufferInfo);
		printf("dst fourcc:\t%s\n",&dest->fourcc);
		printf("dst width:\t%u\t(Crop.width)\n",dest->crop.width);
		printf("dst height:\t%u\t(Crop.height)\n",dest->crop.height);
		printf("dst x:\t%u\t(Crop.x)\n",dest->crop.x);
		printf("dst y:\t%u\t(Crop.y)\n",dest->crop.y);
		printf("dst width:\t%u\t(Image.width)\n",dest_image.width);
		printf("dst height:\t%u\t(Image.height)\n",dest_image.height);
		printf("pitches:\t%u\t(Pitches[0])\n", dest_image.pitches[0]);
		printf("offset: \t%u\t(Offsets[1])\n",dest_image.offsets[1]);
		printf("ali weight:\t%u\n",dest_image.pitches[0]);
		printf("ali height:\t%u\n",dest_image.offsets[1]/dest_image.pitches[0]);	
		printf("handle: \t%lu\n",dest_m_bufferInfo.handle);
		
		printf("Set dst dma buf!\n");
		DmaVideoBuffer *dest_dma_buf;
		VideoBufferInfo dest_buf_info;
		dest_buf_info.init(src->fourcc, 1920, 960, 1920, 960);
		dest_dma_buf = new DmaVideoBuffer(dest_buf_info, dest_m_bufferInfo.handle);
		dest_dma_buf->set_timestamp(dest->timeStamp);
		
		printf("Start stiching!\n");
		//xcam_handle_set_parameters ( XCamHandle *handle, const char *field, ...) need to set some params like fisheye, etc.
		printf("Hanlde ptr:\t%02X\n",stitch_handle);


		printf("Getting usage!\n");
		//char * usage_buf = nullptr;
		//int len = 1024;
		//xcam_handle_get_usage(stitch_handle, usage_buf, &len);


		printf("Release Buffer Handle!\n");
		
		auto srcImageID = image.image_id;
		auto destImageID = dest_image.image_id;

		// Fix the following four releases/destroys.
		// There positions matter a lot.



		printf("Execute!\n");
		xcam_handle_execute (stitch_handle, src_dma_buf, &dest_dma_buf);

        /*****************************************************************/
        status = vaReleaseBufferHandle(m_display->getID(),dest_image.buf);
        printf("release dest buffer handle: %s\n",vaErrorStr(status));

        status = vaReleaseBufferHandle(m_display->getID(),image.buf);
        printf("Release src buffer handle: %s\n",vaErrorStr(status));

        status = vaDestroyImage(m_display->getID(),destImageID);
        printf("Destroy dest image: %s\n",vaErrorStr(status));

        status = vaDestroyImage(m_display->getID(),srcImageID);
        printf("Destroy src image: %s\n",vaErrorStr(status));
        /*****************************************************************/

		printf("Copy parameters to dst!\n");
		const VideoBufferInfo &final_buf_info = dest_dma_buf->get_video_info();
		//assert(dest->surface == dest_dma_buf->get_fd());
		assert(dest_dma_buf->get_timestamp() == src->timeStamp);
		assert(dest_dma_buf->get_timestamp() == dest->timeStamp);
		//dest->timeStamp = src->timeStamp;
		dest->crop = src->crop;
		printf("Final width:\t%u\n",final_buf_info.width);
		dest->crop.width = final_buf_info.width;
		printf("Final height:\t%u\n",final_buf_info.height);
		dest->crop.height = final_buf_info.height;
		//printf("Final fourcc:\t%s\n",final_buf_info.fourcc);
		dest->fourcc = src->fourcc;

        printf("uinit xcam handle\n");
        printf("deallocating dma_buf\n");
//        delete(dest_dma_buf);
//        delete(src_dma_buf);
        //xcam_handle_uinit (stitch_handle);

		printf("********************************************************************************\n");
		printf("****************************** End of Iteration ********************************\n");
		printf("********************************************************************************\n");
		//std::string qwerty;
		//std::cin >> qwerty;

		return YAMI_SUCCESS;
		
	}

};

class TranscodeTest
{
public:
    bool init(int argc, char* argv[])
    {
        if (!processCmdLine(argc, argv, m_cmdParam))
            return false;

        m_display = createVADisplay();
        if (!m_display) {
            printf("create display failed");
            return false;
        }
        if (!createVpp()) {
            ERROR("create vpp failed");
            return false;
        }
        m_input = createInput(m_cmdParam, m_display);
        m_output = createOutput(m_cmdParam, m_display, m_input->getFourcc());
        if (!m_input || !m_output) {
            ERROR("create input or output failed");
            return false;
        }
        m_allocator = createAllocator(m_output, m_display, m_cmdParam.m_encParams.ipPeriod);
        return bool(m_allocator);
    }

    bool run()
    {

        SharedPtr<VideoFrame> src;
        FpsCalc fps;
        uint32_t count = 0;
        while (m_input->read(src)) {
            SharedPtr<VideoFrame> dest = m_allocator->alloc();
            if (!dest) {
                ERROR("failed to get output frame");
                break;
            }
//disable scale for performance measure
//#define DISABLE_SCALE 1
#ifndef DISABLE_SCALE
            YamiStatus status = m_vpp->process(src, dest);
            if (status != YAMI_SUCCESS) {
                ERROR("failed to scale yami return %d", status);
                break;
            }
#else
            dest = src;
#endif

            if(!m_output->output(dest))
                break;
            count++;
            fps.addFrame();
            if(count >= m_cmdParam.frameCount)
                break;
        }
        src.reset();
        m_output->output(src);

        fps.log();

        return true;
    }
private:
    bool createVpp()
    {
        NativeDisplay nativeDisplay;
        nativeDisplay.type = NATIVE_DISPLAY_VA;
        nativeDisplay.handle = (intptr_t)*m_display;
        if (0)	
			m_vpp.reset(createVideoPostProcess(YAMI_VPP_SCALER), releaseVideoPostProcess);
		else
			m_vpp.reset(new StitchPostProcess, releaseVideoPostProcess);
        return m_vpp->setNativeDisplay(nativeDisplay) == YAMI_SUCCESS;
    }

    SharedPtr<VADisplay> m_display;
    SharedPtr<VppInput> m_input;
    SharedPtr<VppOutput> m_output;
    SharedPtr<FrameAllocator> m_allocator;
    SharedPtr<IVideoPostProcess> m_vpp;
    TranscodeParams m_cmdParam;
};

int main(int argc, char** argv)
{

    TranscodeTest trans;
    if (!trans.init(argc, argv)) {
        ERROR("init transcode with command line parameters failed");
        return -1;
    }
    if (!trans.run()){
        ERROR("run transcode failed");
        return -1;
    }
    printf("transcode done\n");
    return  0;

}
