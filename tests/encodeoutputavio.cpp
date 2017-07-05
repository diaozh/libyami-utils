#include "encodeoutputavio.h"

EncodeOutputH264Stream::EncodeOutputH264Stream():m_sp(NULL)
{
}

EncodeOutputH264Stream::~EncodeOutputH264Stream()
{
    if (m_sp)
        avio_close(m_sp);
}

bool EncodeOutputH264Stream::init(const char* outputFileName, int width, int height, int fps)
{
    int ret;
    avformat_network_init();
    ret = avio_open(&m_sp, outputFileName, AVIO_FLAG_WRITE);
    if (ret < 0) {
        fprintf(stderr, "Error opening the url: ");
        return false;
    }
    return true;
}

bool EncodeOutputH264Stream::write(void* data, int size)
{
    avio_write(m_sp, (const unsigned char*)data, size);
    return true;
}

const char* EncodeOutputH264Stream::getMimeType()
{
    return YAMI_MIME_H264;
}
