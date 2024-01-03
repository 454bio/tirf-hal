#include <chrono>
#include <cstring>

// TODO: The include_directories punning is breaking pragma once
#include "core/libcamera_app.hpp"
#include "image/image.hpp"

#include "tiff.hpp"

#include "image_buffer.hpp"

OwningImageBuffer::OwningImageBuffer()
    : buffer_(std::vector<std::vector<uint8_t>>(1, OwningImageBuffer::make_preallocated_vector()))
{}

void OwningImageBuffer::replace_with_libcamera_spans(
    const std::vector<libcamera::Span<uint8_t>> &mem,
    const StreamInfo &info
)
{
    info_ = info;

    buffer_.resize(mem.size());
    image_.resize(mem.size());
    for (size_t mem_part_index = 0; mem_part_index < mem.size(); ++mem_part_index)
    {
        auto &mem_part = mem[mem_part_index];
        auto &buffer_part = buffer_[mem_part_index];

        if (buffer_part.size() < mem_part.size())
        {
            LOG(1, "Either no preallocated buffer was provided, or it was too small. Consider creating one or increasing its size to avoid a 30ms allocation delay.");
            LOG(1, "buffer_part.size() = " << buffer_part.size() << "; mem_part.size() = " << mem_part.size());
        }
        // Make sure we have somewhere to copy to...
        buffer_part.resize(mem_part.size());
        // ... actually do the copy...
        std::memcpy(buffer_part.data(), mem_part.data(), mem_part.size_bytes());
        // ... and make the interface.
        image_[mem_part_index] = libcamera::Span<uint8_t>(buffer_part.data(), buffer_part.size());
    }
}

OwningImageBuffer::OwningImageBuffer(
    const std::vector<libcamera::Span<uint8_t>> &mem,
    const StreamInfo &info,
    std::vector<std::vector<uint8_t>> &&preallocated
)
{
    // `preallocated` is a *hint*. If it's wrong, we'll reallocate as needed.
    buffer_ = {std::move(preallocated)};

    // Copy the acquired image to the buffer
    replace_with_libcamera_spans(mem, info);
}

OwningImageBuffer::OwningImageBuffer(const OwningImageBuffer &other)
    : buffer_(other.buffer_)
    , info_(other.info_)
{
    LOG(1, "*COPYING* an OwningImageBuffer. This may be adding an unnecessary delay. Try to move these instead -- did you forget a && or a std::move()?");

    // buffer_ is now different memory, so we need to recreate the interface from scratch.
    image_.resize(buffer_.size());
    for (size_t buffer_index = 0; buffer_index < buffer_.size(); ++buffer_index)
    {
        auto &buffer_part = buffer_[buffer_index];
        image_[buffer_index] = libcamera::Span<uint8_t>(buffer_part.data(), buffer_part.size());
    }
}

void OwningImageBuffer::save_tiff(std::ostream &tiff_stream, bool output_color, unsigned int downsampling) const
{
    if (output_color)
    {
        if (downsampling != 1)
        {
            LOG(1, "Downsampling requested on a color image, which is unsupported.");
        }
        tiff_save_color(image_, info_, tiff_stream);
    }
    else
    {
        tiff_save_gray_debayered(image_, info_, tiff_stream, downsampling);
    }
}

constexpr static size_t IMAGE_VECTOR_SIZE = 30 << 20;

std::vector<uint8_t> OwningImageBuffer::make_preallocated_vector()
{
    return std::vector<uint8_t>(IMAGE_VECTOR_SIZE);
}
