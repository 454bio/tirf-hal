#pragma once

inline size_t align_ceil_32(size_t unaligned)
{
	return (unaligned & ~((size_t) 0x1F)) + ((unaligned & 0x1F) ? 0x20 : 0);
}

void tiff_save_color(std::vector<libcamera::Span<uint8_t>> const &mem, StreamInfo const &info,
			         std::ostream &tiff_stream);
void tiff_save_gray_debayered(std::vector<libcamera::Span<uint8_t>> const &mem, StreamInfo const &info,
			                  std::ostream &tiff_stream, unsigned int downsampling = 1);
