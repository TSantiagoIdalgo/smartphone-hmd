#include "frame_capture.h"

#include <dxgi1_2.h>

#include "driver_log.h"

using Microsoft::WRL::ComPtr;

namespace {

// Returns 0 for RGBA byte order, 1 for BGRA, -1 for formats we cannot convert.
int ByteOrder(DXGI_FORMAT format)
{
    switch (format) {
        case DXGI_FORMAT_R8G8B8A8_TYPELESS:
        case DXGI_FORMAT_R8G8B8A8_UNORM:
        case DXGI_FORMAT_R8G8B8A8_UNORM_SRGB:
            return 0;
        case DXGI_FORMAT_B8G8R8A8_TYPELESS:
        case DXGI_FORMAT_B8G8R8A8_UNORM:
        case DXGI_FORMAT_B8G8R8A8_UNORM_SRGB:
        case DXGI_FORMAT_B8G8R8X8_TYPELESS:
        case DXGI_FORMAT_B8G8R8X8_UNORM:
        case DXGI_FORMAT_B8G8R8X8_UNORM_SRGB:
            return 1;
        default:
            return -1;
    }
}

}  // namespace

bool FrameCapture::Init(LUID* adapter_luid)
{
    ComPtr<IDXGIFactory1> factory;
    if (FAILED(CreateDXGIFactory1(IID_PPV_ARGS(&factory)))) {
        DriverLog("CreateDXGIFactory1 failed");
        return false;
    }
    ComPtr<IDXGIAdapter1> adapter;
    if (FAILED(factory->EnumAdapters1(0, &adapter))) {
        DriverLog("No GPU found");
        return false;
    }
    DXGI_ADAPTER_DESC1 desc{};
    adapter->GetDesc1(&desc);
    *adapter_luid = desc.AdapterLuid;

    const HRESULT hr = D3D11CreateDevice(adapter.Get(), D3D_DRIVER_TYPE_UNKNOWN, nullptr, D3D11_CREATE_DEVICE_BGRA_SUPPORT,
                                         nullptr, 0, D3D11_SDK_VERSION, &device_, nullptr, &context_);
    if (FAILED(hr)) {
        DriverLog("D3D11CreateDevice failed: 0x%08lx", static_cast<unsigned long>(hr));
        return false;
    }
    DriverLog("Capturing frames on GPU: %ls", desc.Description);
    return true;
}

bool FrameCapture::Copy(uint64_t shared_texture_handle)
{
    pending_ = false;
    if (shared_textures_.size() > 16) {
        shared_textures_.clear();
    }
    ComPtr<ID3D11Texture2D>& texture = shared_textures_[shared_texture_handle];
    if (!texture) {
        const HRESULT hr = device_->OpenSharedResource(reinterpret_cast<HANDLE>(static_cast<uintptr_t>(shared_texture_handle)),
                                                       IID_PPV_ARGS(&texture));
        if (FAILED(hr)) {
            DriverLog("OpenSharedResource failed: 0x%08lx", static_cast<unsigned long>(hr));
            shared_textures_.erase(shared_texture_handle);
            return false;
        }
    }

    D3D11_TEXTURE2D_DESC desc{};
    texture->GetDesc(&desc);
    const int order = ByteOrder(desc.Format);
    if (order < 0) {
        if (!warned_format_) {
            DriverLog("Unsupported compositor texture format %d", static_cast<int>(desc.Format));
            warned_format_ = true;
        }
        return false;
    }
    bgra_ = order == 1;

    if (!staging_ || staging_desc_.Width != desc.Width || staging_desc_.Height != desc.Height ||
        staging_desc_.Format != desc.Format) {
        D3D11_TEXTURE2D_DESC staging_desc{};
        staging_desc.Width = desc.Width;
        staging_desc.Height = desc.Height;
        staging_desc.MipLevels = 1;
        staging_desc.ArraySize = 1;
        staging_desc.Format = desc.Format;
        staging_desc.SampleDesc.Count = 1;
        staging_desc.Usage = D3D11_USAGE_STAGING;
        staging_desc.CPUAccessFlags = D3D11_CPU_ACCESS_READ;
        staging_.Reset();
        if (FAILED(device_->CreateTexture2D(&staging_desc, nullptr, &staging_))) {
            DriverLog("Could not create the staging texture");
            return false;
        }
        staging_desc_ = staging_desc;
        DriverLog("Capturing %ux%u frames, format %d", desc.Width, desc.Height, static_cast<int>(desc.Format));
    }

    context_->CopySubresourceRegion(staging_.Get(), 0, 0, 0, 0, texture.Get(), 0, nullptr);
    context_->Flush();
    pending_ = true;
    return true;
}

bool FrameCapture::Read(const ReadFunction& read)
{
    if (!pending_) {
        return false;
    }
    pending_ = false;
    D3D11_MAPPED_SUBRESOURCE mapped{};
    if (FAILED(context_->Map(staging_.Get(), 0, D3D11_MAP_READ, 0, &mapped))) {
        return false;
    }
    read(static_cast<const uint8_t*>(mapped.pData), static_cast<int>(mapped.RowPitch),
         static_cast<int>(staging_desc_.Width), static_cast<int>(staging_desc_.Height), bgra_);
    context_->Unmap(staging_.Get(), 0);
    return true;
}
