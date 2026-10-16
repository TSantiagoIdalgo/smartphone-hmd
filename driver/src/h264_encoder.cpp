#include "h264_encoder.h"

#include <chrono>
#include <cstring>
#include <thread>

#include <mfapi.h>
#include <mferror.h>

#include "driver_log.h"
#include "video_protocol.h"

using Microsoft::WRL::ComPtr;

namespace {

bool SetCodecValue(ICodecAPI* codec, const GUID& property, UINT32 value)
{
    VARIANT v;
    VariantInit(&v);
    v.vt = VT_UI4;
    v.ulVal = value;
    return SUCCEEDED(codec->SetValue(&property, &v));
}

// 0 if the encoder does not say.
UINT32 GetCodecValue(ICodecAPI* codec, const GUID& property)
{
    VARIANT v;
    VariantInit(&v);
    if (FAILED(codec->GetValue(&property, &v))) {
        return 0;
    }
    const UINT32 value = v.vt == VT_UI4 ? v.ulVal : 0;
    VariantClear(&v);
    return value;
}

void SetCodecFlag(ICodecAPI* codec, const GUID& property, bool value)
{
    VARIANT v;
    VariantInit(&v);
    v.vt = VT_BOOL;
    v.boolVal = value ? VARIANT_TRUE : VARIANT_FALSE;
    codec->SetValue(&property, &v);
}

ComPtr<IMFActivate> FindEncoder(UINT32 flags)
{
    MFT_REGISTER_TYPE_INFO input{MFMediaType_Video, MFVideoFormat_NV12};
    MFT_REGISTER_TYPE_INFO output{MFMediaType_Video, MFVideoFormat_H264};
    IMFActivate** activates = nullptr;
    UINT32 count = 0;
    ComPtr<IMFActivate> first;
    if (SUCCEEDED(MFTEnumEx(MFT_CATEGORY_VIDEO_ENCODER, flags | MFT_ENUM_FLAG_SORTANDFILTER, &input, &output,
                            &activates, &count))) {
        for (UINT32 i = 0; i < count; ++i) {
            if (i == 0) {
                first = activates[i];
            }
            activates[i]->Release();
        }
        CoTaskMemFree(activates);
    }
    return first;
}

}  // namespace

H264Encoder::~H264Encoder()
{
    Shutdown();
}

bool H264Encoder::Init(int width, int height, int fps, int bitrate)
{
    Shutdown();
    width_ = width;
    height_ = height;
    fps_ = fps;

    ComPtr<IMFActivate> activate = FindEncoder(MFT_ENUM_FLAG_HARDWARE);
    const bool hardware = activate != nullptr;
    if (!activate) {
        activate = FindEncoder(MFT_ENUM_FLAG_SYNCMFT);
    }
    if (!activate || FAILED(activate->ActivateObject(IID_PPV_ARGS(&transform_)))) {
        DriverLog("No H.264 encoder available");
        return false;
    }

    ComPtr<IMFAttributes> attributes;
    if (SUCCEEDED(transform_->GetAttributes(&attributes))) {
        UINT32 is_async = 0;
        attributes->GetUINT32(MF_TRANSFORM_ASYNC, &is_async);
        if (is_async) {
            attributes->SetUINT32(MF_TRANSFORM_ASYNC_UNLOCK, TRUE);
            async_ = SUCCEEDED(transform_.As(&events_));
        }
        attributes->SetUINT32(MF_LOW_LATENCY, TRUE);
    }

    if (transform_->GetStreamIDs(1, &input_id_, 1, &output_id_) == E_NOTIMPL) {
        input_id_ = 0;
        output_id_ = 0;
    }

    if (SUCCEEDED(transform_->QueryInterface(IID_ICodecAPI, reinterpret_cast<void**>(codec_.ReleaseAndGetAddressOf()))) &&
        codec_) {
        SetCodecFlag(codec_.Get(), CODECAPI_AVLowLatencyMode, true);
        SetCodecValue(codec_.Get(), CODECAPI_AVEncCommonRateControlMode, eAVEncCommonRateControlMode_CBR);
        SetCodecValue(codec_.Get(), CODECAPI_AVEncCommonMeanBitRate, static_cast<UINT32>(bitrate));
        // Keyframes only when the phone asks for one: when it connects and after a lost packet. A
        // keyframe is several times bigger than other frames, so periodic ones made the picture
        // blur and catch up every couple of seconds. Encoders cap this differently, so try smaller.
        for (const UINT32 frames : {static_cast<UINT32>(fps * 600), 1000u, static_cast<UINT32>(fps * 10)}) {
            if (SetCodecValue(codec_.Get(), CODECAPI_AVEncMPVGOPSize, frames)) {
                break;
            }
        }
        SetCodecValue(codec_.Get(), CODECAPI_AVEncMPVDefaultBPictureCount, 0);
    }

    ComPtr<IMFMediaType> output_type;
    MFCreateMediaType(&output_type);
    output_type->SetGUID(MF_MT_MAJOR_TYPE, MFMediaType_Video);
    output_type->SetGUID(MF_MT_SUBTYPE, MFVideoFormat_H264);
    output_type->SetUINT32(MF_MT_AVG_BITRATE, static_cast<UINT32>(bitrate));
    MFSetAttributeSize(output_type.Get(), MF_MT_FRAME_SIZE, static_cast<UINT32>(width), static_cast<UINT32>(height));
    MFSetAttributeRatio(output_type.Get(), MF_MT_FRAME_RATE, static_cast<UINT32>(fps), 1);
    MFSetAttributeRatio(output_type.Get(), MF_MT_PIXEL_ASPECT_RATIO, 1, 1);
    output_type->SetUINT32(MF_MT_INTERLACE_MODE, MFVideoInterlace_Progressive);
    output_type->SetUINT32(MF_MT_MPEG2_PROFILE, eAVEncH264VProfile_Base);
    HRESULT hr = transform_->SetOutputType(output_id_, output_type.Get(), 0);
    if (FAILED(hr)) {
        DriverLog("H.264 SetOutputType failed: 0x%08lx", static_cast<unsigned long>(hr));
        return false;
    }

    ComPtr<IMFMediaType> input_type;
    MFCreateMediaType(&input_type);
    input_type->SetGUID(MF_MT_MAJOR_TYPE, MFMediaType_Video);
    input_type->SetGUID(MF_MT_SUBTYPE, MFVideoFormat_NV12);
    MFSetAttributeSize(input_type.Get(), MF_MT_FRAME_SIZE, static_cast<UINT32>(width), static_cast<UINT32>(height));
    MFSetAttributeRatio(input_type.Get(), MF_MT_FRAME_RATE, static_cast<UINT32>(fps), 1);
    MFSetAttributeRatio(input_type.Get(), MF_MT_PIXEL_ASPECT_RATIO, 1, 1);
    input_type->SetUINT32(MF_MT_INTERLACE_MODE, MFVideoInterlace_Progressive);
    hr = transform_->SetInputType(input_id_, input_type.Get(), 0);
    if (FAILED(hr)) {
        DriverLog("H.264 SetInputType failed: 0x%08lx", static_cast<unsigned long>(hr));
        return false;
    }

    MFT_OUTPUT_STREAM_INFO info{};
    transform_->GetOutputStreamInfo(output_id_, &info);
    provides_samples_ = (info.dwFlags & (MFT_OUTPUT_STREAM_PROVIDES_SAMPLES | MFT_OUTPUT_STREAM_CAN_PROVIDE_SAMPLES)) != 0;
    output_size_ = info.cbSize > 0 ? info.cbSize : static_cast<DWORD>(width * height * 3 / 2);
    ReadSequenceHeader();

    transform_->ProcessMessage(MFT_MESSAGE_NOTIFY_BEGIN_STREAMING, 0);
    transform_->ProcessMessage(MFT_MESSAGE_NOTIFY_START_OF_STREAM, 0);
    DriverLog("H.264 encoder ready: %dx%d at %d fps, %d kbit/s, %s, %s, keyframe every %u frames", width, height,
              fps, bitrate / 1000, hardware ? "hardware" : "software", async_ ? "async" : "sync",
              codec_ ? GetCodecValue(codec_.Get(), CODECAPI_AVEncMPVGOPSize) : 0u);
    return true;
}

void H264Encoder::Shutdown()
{
    if (transform_) {
        transform_->ProcessMessage(MFT_MESSAGE_NOTIFY_END_OF_STREAM, 0);
        transform_->ProcessMessage(MFT_MESSAGE_NOTIFY_END_STREAMING, 0);
    }
    codec_.Reset();
    events_.Reset();
    transform_.Reset();
    async_ = false;
    need_input_ = 0;
    sequence_header_.clear();
}

void H264Encoder::ReadSequenceHeader()
{
    ComPtr<IMFMediaType> type;
    if (FAILED(transform_->GetOutputCurrentType(output_id_, &type))) {
        return;
    }
    UINT32 size = 0;
    if (SUCCEEDED(type->GetBlobSize(MF_MT_MPEG_SEQUENCE_HEADER, &size)) && size > 0) {
        sequence_header_.resize(size);
        type->GetBlob(MF_MT_MPEG_SEQUENCE_HEADER, sequence_header_.data(), size, nullptr);
    }
}

bool H264Encoder::Encode(const uint8_t* nv12, int64_t time_100ns, bool force_keyframe, const OutputFunction& output)
{
    if (!transform_) {
        return false;
    }
    const DWORD size = static_cast<DWORD>(width_ * height_ * 3 / 2);
    ComPtr<IMFMediaBuffer> buffer;
    ComPtr<IMFSample> sample;
    if (FAILED(MFCreateMemoryBuffer(size, &buffer)) || FAILED(MFCreateSample(&sample))) {
        return false;
    }
    BYTE* destination = nullptr;
    buffer->Lock(&destination, nullptr, nullptr);
    std::memcpy(destination, nv12, size);
    buffer->Unlock();
    buffer->SetCurrentLength(size);
    sample->AddBuffer(buffer.Get());
    sample->SetSampleTime(time_100ns);
    sample->SetSampleDuration(10000000 / fps_);

    if (force_keyframe && codec_) {
        SetCodecValue(codec_.Get(), CODECAPI_AVEncVideoForceKeyFrame, 1);
    }

    if (!async_) {
        if (FAILED(transform_->ProcessInput(input_id_, sample.Get(), 0))) {
            return false;
        }
        while (ProcessOutput(output)) {
        }
        return true;
    }

    // Asynchronous (hardware) encoders tell us through events when they want input or have output.
    while (HandleEvent(false, output)) {
    }
    while (need_input_ == 0) {
        if (!HandleEvent(true, output)) {
            return false;
        }
    }
    if (FAILED(transform_->ProcessInput(input_id_, sample.Get(), 0))) {
        return false;
    }
    --need_input_;
    // Wait up to one frame for this frame's output. Hardware encoders often ask for the next input
    // before handing it over; stopping at that request would hold every frame back until the next.
    const int outputs_before = outputs_;
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::microseconds(1000000 / fps_);
    while (outputs_ == outputs_before && std::chrono::steady_clock::now() < deadline) {
        if (!HandleEvent(false, output)) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    }
    return true;
}

bool H264Encoder::HandleEvent(bool wait, const OutputFunction& output)
{
    ComPtr<IMFMediaEvent> event;
    if (FAILED(events_->GetEvent(wait ? 0 : MF_EVENT_FLAG_NO_WAIT, &event))) {
        return false;
    }
    MediaEventType type = MEUnknown;
    event->GetType(&type);
    if (type == METransformNeedInput) {
        ++need_input_;
    } else if (type == METransformHaveOutput) {
        ProcessOutput(output);
    }
    return true;
}

bool H264Encoder::ProcessOutput(const OutputFunction& output)
{
    MFT_OUTPUT_DATA_BUFFER data{};
    data.dwStreamID = output_id_;
    ComPtr<IMFSample> own_sample;
    if (!provides_samples_) {
        ComPtr<IMFMediaBuffer> buffer;
        if (FAILED(MFCreateMemoryBuffer(output_size_, &buffer)) || FAILED(MFCreateSample(&own_sample))) {
            return false;
        }
        own_sample->AddBuffer(buffer.Get());
        data.pSample = own_sample.Get();
    }

    DWORD status = 0;
    const HRESULT hr = transform_->ProcessOutput(0, 1, &data, &status);
    if (data.pEvents) {
        data.pEvents->Release();
    }
    if (hr == MF_E_TRANSFORM_STREAM_CHANGE) {
        ComPtr<IMFMediaType> type;
        if (SUCCEEDED(transform_->GetOutputAvailableType(output_id_, 0, &type))) {
            transform_->SetOutputType(output_id_, type.Get(), 0);
            ReadSequenceHeader();
        }
        return true;
    }
    if (FAILED(hr)) {
        return false;  // usually MF_E_TRANSFORM_NEED_MORE_INPUT
    }

    ComPtr<IMFSample> sample;
    if (provides_samples_) {
        sample.Attach(data.pSample);
    } else {
        sample = own_sample;
    }
    if (!sample) {
        return false;
    }

    UINT32 clean_point = 0;
    sample->GetUINT32(MFSampleExtension_CleanPoint, &clean_point);
    const bool keyframe = clean_point != 0;
    LONGLONG time_100ns = -1;
    sample->GetSampleTime(&time_100ns);
    ++outputs_;

    ComPtr<IMFMediaBuffer> buffer;
    if (FAILED(sample->ConvertToContiguousBuffer(&buffer))) {
        return false;
    }
    BYTE* bytes = nullptr;
    DWORD length = 0;
    if (SUCCEEDED(buffer->Lock(&bytes, nullptr, &length))) {
        // The phone's decoder needs the SPS/PPS before each keyframe; some encoders only put them
        // in the media type.
        if (keyframe && !sequence_header_.empty() && !video::StartsWithSps(bytes, length)) {
            keyframe_buffer_.assign(sequence_header_.begin(), sequence_header_.end());
            keyframe_buffer_.insert(keyframe_buffer_.end(), bytes, bytes + length);
            output(keyframe_buffer_.data(), keyframe_buffer_.size(), true, time_100ns);
        } else {
            output(bytes, length, keyframe, time_100ns);
        }
        buffer->Unlock();
    }
    return true;
}
