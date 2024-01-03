#pragma once

#include <chrono>
#include <vector>

#include <libcamera/controls.h>

#include "core/stream_info.hpp"

// Buffer for temporary image storage beyond what the libcamera interface allows.
class OwningImageBuffer
{
public:
    // Construct an empty `OwningImageBuffer` of size `IMAGE_VECTOR_SIZE`.
    OwningImageBuffer();

    // Construct an `OwningImageBuffer` by copying from the libcamera request buffer.
    OwningImageBuffer(
        const std::vector<libcamera::Span<uint8_t>> &mem,
        const StreamInfo &info,
        std::vector<std::vector<uint8_t>> &&preallocated = std::vector<std::vector<uint8_t>>());

    // Copy constructor, since we need to recreate the Span interface.
    OwningImageBuffer(const OwningImageBuffer &other);
    // Moving does not invalidate references, so the default is fine.
    OwningImageBuffer(OwningImageBuffer &&other) = default;

    // Replace our contents using a libcamera request buffer.
    void replace_with_libcamera_spans(
        const std::vector<libcamera::Span<uint8_t>> &mem,
        const StreamInfo &info
    );

    void save_tiff(std::ostream &tiff_stream, bool output_color, unsigned int downsampling = 1) const;

    // Pre-allocated buffer for the images so we don't have to allocate during the capture itself.
    // This is likely to be slightly too big, but there is not a significant time penalty for resizing down.
    // This is getting awfully close to implementing an allocator...
    static std::vector<uint8_t> make_preallocated_vector();

private:
    // libcamera-compatible image interface.
    // These typically only have one element and are weak pointers.
    std::vector<libcamera::Span<uint8_t>> image_;

    // Backing storage for the image.
    std::vector<std::vector<uint8_t>> buffer_;

    StreamInfo info_;
};
