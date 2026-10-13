#pragma once

#include <cstdint>
#include <functional>
#include <map>

#include <d3d11.h>
#include <wrl/client.h>

// Copies the frames SteamVR's compositor presents (shared D3D11 textures) into CPU memory.
class FrameCapture {
public:
    // Creates a D3D11 device on the first GPU and reports that GPU's id, which SteamVR must use too.
    bool Init(LUID* adapter_luid);

    // Starts copying the shared texture to a CPU-readable one. Call from the compositor's Present().
    bool Copy(uint64_t shared_texture_handle);

    using ReadFunction = std::function<void(const uint8_t* pixels, int pitch, int width, int height, bool bgra)>;
    // Waits for the copy and gives `read` the pixels. Returns false if there is nothing to read.
    bool Read(const ReadFunction& read);

private:
    Microsoft::WRL::ComPtr<ID3D11Device> device_;
    Microsoft::WRL::ComPtr<ID3D11DeviceContext> context_;
    Microsoft::WRL::ComPtr<ID3D11Texture2D> staging_;
    std::map<uint64_t, Microsoft::WRL::ComPtr<ID3D11Texture2D>> shared_textures_;
    D3D11_TEXTURE2D_DESC staging_desc_{};
    bool pending_ = false;
    bool bgra_ = false;
    bool warned_format_ = false;
};
