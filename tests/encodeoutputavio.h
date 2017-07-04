#ifndef encodeoutputavio_h
#define encodeoutputavio_h

extern "C" {
#include <libavformat/avio.h>
}

class EncodeOutputH264Stream : public EncodeOutput {
public:
    EncodeOutputH264Stream();
    ~EncodeOutputH264Stream();
    virtual const char* getMimeType();
    virtual bool write(void* data, int size);
protected:
    virtual bool init(const char* outputFileName, int width, int height, int fps = 30);
private:
    AVIOContext *m_sp;
};

#endif
