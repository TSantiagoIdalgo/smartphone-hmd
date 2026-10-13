#pragma once

#include <cstdint>
#include <functional>
#include <vector>

#include <mftransform.h>
#include <strmif.h>
#include <codecapi.h>
#include <wrl/client.h>

// H.264 encoder on top of Windows Media Foundation. Prefers the GPU's hardware encoder and falls
// back to Microsoft's software encoder. Use it from a single thread.
class H264Encoder {
public:
    ~H264Encoder();

    bool Init(int width, int height, int fps, int bitrate);
    void Shutdown();

    using OutputFunction = std::function<void(const uint8_t* data, size_t size, bool keyframe)>;
    // Encodes one NV12 frame (width*height*3/2 bytes). Encoded frames go to `output`, usually
    // this frame's, but an encoder may hand it over on a later call.
    bool Encode(const uint8_t* nv12, int64_t time_100ns, bool force_keyframe, const OutputFunction& output);

    int Width() const { return width_; }
    int Height() const { return height_; }

private:
    bool HandleEvent(bool wait, const OutputFunction& output);
    bool ProcessOutput(const OutputFunction& output);
    void ReadSequenceHeader();

    Microsoft::WRL::ComPtr<IMFTransform> transform_;
    Microsoft::WRL::ComPtr<IMFMediaEventGenerator> events_;
    Microsoft::WRL::ComPtr<ICodecAPI> codec_;
    DWORD input_id_ = 0;
    DWORD output_id_ = 0;
    bool async_ = false;
    bool provides_samples_ = false;
    DWORD output_size_ = 0;
    int need_input_ = 0;
    int width_ = 0;
    int height_ = 0;
    int fps_ = 60;
    std::vector<uint8_t> sequence_header_;
    std::vector<uint8_t> keyframe_buffer_;
};
