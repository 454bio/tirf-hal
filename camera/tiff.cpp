#include <libcamera/control_ids.h>
#include <libcamera/formats.h>
#include <linux/videodev2.h>

#include <tiffio.h>
#include <tiffio.hxx>

#include "image/image.hpp"

#include "core/stream_info.hpp"
#include "utils/c_utils.hpp"

#include "tiff.hpp"

using namespace libcamera;

static constexpr size_t SAMPLES_PER_PIXEL_COLOR = 3;
static constexpr size_t SAMPLES_PER_PIXEL_GRAY = 1;

static constexpr libcamera::PixelFormat VC_FORMAT_Y12P(v4l2_fourcc('Y', '1', '2', 'P'));

// Naively save a libcamera buffer to TIFF, preserving color by averaging the two greens.
void tiff_save_color(
    std::vector<libcamera::Span<uint8_t>> const &mem, StreamInfo const &info, std::ostream &tiff_stream)
{
    if (info.pixel_format != libcamera::formats::SGRBG12_CSI2P)
    {
        throw std::runtime_error("Unsupported Bayer format");
    }

    // Assume that libcamera always produces a `mem` that consists of one element with all of the data
    uint8_t *input_buffer = mem[0].data();

    TIFF* tif = TIFFStreamOpen("", &tiff_stream);
    CleanupHelper tif_close([&tif]()
    {
        TIFFClose(tif);
    });

    // Rather than doing any interpolation, we are taking every group of 4 elements (RGGB) and converting it into a pixel.
    uint32_t width = info.width / 2;
    uint32_t height = info.height / 2;

    TIFFSetField(tif, TIFFTAG_IMAGEWIDTH, width);
    TIFFSetField(tif, TIFFTAG_IMAGELENGTH, height);
    TIFFSetField(tif, TIFFTAG_BITSPERSAMPLE, 16);
    TIFFSetField(tif, TIFFTAG_SAMPLESPERPIXEL, SAMPLES_PER_PIXEL_COLOR);
    TIFFSetField(tif, TIFFTAG_PHOTOMETRIC, PHOTOMETRIC_RGB);
    TIFFSetField(tif, TIFFTAG_ORIENTATION, ORIENTATION_TOPLEFT);
    TIFFSetField(tif, TIFFTAG_PLANARCONFIG, PLANARCONFIG_CONTIG);
    TIFFSetField(tif, TIFFTAG_ROWSPERSTRIP, 1);

    // TODO: More metadata?

    uint8_t *dual_line_start;
    size_t used_bytes_per_line = width * SAMPLES_PER_PIXEL_COLOR;

    // libcamera 32-bit aligns its rows
    size_t bytes_per_line = align_ceil_32(used_bytes_per_line);

    for (size_t l=0; l<height; l++) // Line index
    {
        // We write two lines each loop so we can reference the subpixels on the following line
        std::vector<uint16_t> output_line_buffer(used_bytes_per_line * 2);
    
        // we want to look at lines 2*l and 2*l+1, which is the following:
        //memcpy(dual_line_buffer, &input_buffer[2*l*BYTES_PER_LINE], BYTES_PER_LINE*2)
        //now dual_line_buffer is set up as follows:
        dual_line_start = input_buffer + 2*l*bytes_per_line; //&input_buffer[2*l*BYTES_PER_LINE]; //
        for (size_t i=0; i < used_bytes_per_line; i=i+SAMPLES_PER_PIXEL_COLOR) // Pixel index in line
        {
            std::array<uint16_t, SAMPLES_PER_PIXEL_COLOR> current_pixel;

            uint16_t green_pixel_1 = (*(i+dual_line_start) << 4) | (*(i+dual_line_start+2) & 0x0F);
            uint16_t green_pixel_2 = (*(i+dual_line_start+1+bytes_per_line) << 4) | ((*(i+dual_line_start+2+bytes_per_line) >> 4) & 0x0F);

            // Red
            current_pixel[0] = ( (*(i+dual_line_start+1) << 4) | ((*(i+dual_line_start+2) >> 4) & 0x0F) ) << 4;

            // Blue
            current_pixel[2] = ( (*(i+dual_line_start+bytes_per_line) << 4) | (*(i+dual_line_start+2+bytes_per_line) & 0x0F) ) << 4;

            // Green is the sum (not average) of the two green elements.
            // Note that it is impossible to re-bucket the green elements after saving the image this way.
            current_pixel[1] = (green_pixel_1 << 3) + (green_pixel_2 << 3);

            std::copy(current_pixel.begin(), current_pixel.end(), output_line_buffer.begin()+i);
        }

        CHECK_RC(TIFFWriteScanline(tif, output_line_buffer.data(), l, 0), "Error writing scanline");
    }
}

// Naively save a libcamera buffer to TIFF, assuming that it was generated by a debayered but otherwise standard Pi camera.
void tiff_save_gray_debayered(std::vector<libcamera::Span<uint8_t>> const &mem, StreamInfo const &info, std::ostream &tiff_stream, unsigned int downsampling)
{
    // This should work across all of the packed 12-bit formats.
    switch (info.pixel_format)
    {
        case libcamera::formats::SRGGB12_CSI2P:
        case libcamera::formats::SGRBG12_CSI2P:
        case libcamera::formats::SGBRG12_CSI2P:
        case libcamera::formats::SBGGR12_CSI2P:
        case VC_FORMAT_Y12P:
            break;
        default:
            throw std::runtime_error("Unsupported Bayer format");
            break;
    }

    // Assume that libcamera always produces a `mem` that consists of one element with all of the data
    uint8_t *input_buffer = mem[0].data();

    TIFF* tif = TIFFStreamOpen("", &tiff_stream);
    CleanupHelper tif_close([&tif]()
    {
        TIFFClose(tif);
    });

    // Since we're using each pixel directly without any channel assignment, just use the raw dimensions.
    uint32_t input_width = info.width;
    uint32_t input_height = info.height;
    // 2 color "subpixels" every 3 bytes. See diagram below.
    size_t used_bytes_per_line = input_width / 2 * SAMPLES_PER_PIXEL_COLOR;

    // libcamera 32-bit aligns its rows
    size_t bytes_per_line = align_ceil_32(used_bytes_per_line);

    uint32_t output_width = input_width / downsampling + bool(input_width % downsampling);
    uint32_t output_height = input_height / downsampling + bool(input_height % downsampling);

    TIFFSetField(tif, TIFFTAG_IMAGEWIDTH, output_width);
    TIFFSetField(tif, TIFFTAG_IMAGELENGTH, output_height);
    TIFFSetField(tif, TIFFTAG_BITSPERSAMPLE, 16);
    TIFFSetField(tif, TIFFTAG_SAMPLESPERPIXEL, SAMPLES_PER_PIXEL_GRAY);
    TIFFSetField(tif, TIFFTAG_PHOTOMETRIC, PHOTOMETRIC_MINISBLACK);
    TIFFSetField(tif, TIFFTAG_ORIENTATION, ORIENTATION_TOPLEFT);
    TIFFSetField(tif, TIFFTAG_PLANARCONFIG, PLANARCONFIG_CONTIG);
    TIFFSetField(tif, TIFFTAG_ROWSPERSTRIP, 1);

    /*
    The pixel data looks something like this:
        GG RR GR|GG RR GR|GG RR GR...
        BB GG BG|BB GG BG|BB GG BG...
        GG RR GR|GG RR GR|GG RR GR...
        BB GG BG|BB GG BG|BB GG BG...
        GG RR GR|GG RR GR|GG RR GR...
        BB GG BG|BB GG BG|BB GG BG...
        ...

    Each column represents 4 bits.
    The exact colors will differ depending on which packed 12-bit format we receive.
    Spaces denote a byte boundary, and pipes denote a 3-byte boundary.
    We need to iterate over blocks 3 bytes wide to retrieve two adjacent color subpixels.
    We will interpret each as one full grayscale pixel.
    */

    uint8_t *line_start;
    // Skip strides if we're downsampling.
    size_t stride = (downsampling == 1) ? SAMPLES_PER_PIXEL_COLOR : SAMPLES_PER_PIXEL_COLOR * downsampling / 2;
    for (size_t l = 0; l < input_height; l += downsampling)
    {
        std::vector<uint16_t> output_line_buffer(output_width);
        std::vector<uint16_t>::iterator output_pixel_iterator = output_line_buffer.begin();

        line_start = input_buffer + l*bytes_per_line;
        // Need to iterate over blocks 3 bytes wide to extract individual pixels, but this time we don't care where they come from.
        for (size_t i = 0; i < used_bytes_per_line; i += stride)
        {
            *(output_pixel_iterator++) =
                ( // Extract the first pixel according to the above...
                    ((*(line_start+i+0) & 0xFF) << 4) |
                    ((*(line_start+i+2) & 0xF0) >> 4)
                ); // << 4; // ... and shift it into a "16-bit" value.
            if (downsampling == 1)
            {
                // Only want the second pixel if we're not downsampling.
                // Only supporting even values here dramatically simplifies the implementation.
                *(output_pixel_iterator++) =
                    ( // Extract the second pixel according to the above...
                        ((*(line_start+i+1) & 0xFF) << 4) |
                        ((*(line_start+i+2) & 0x0F) >> 0)
                    ); // << 4; // ... and shift it into a "16-bit" value.
            }
        }

        CHECK_RC(TIFFWriteScanline(tif, output_line_buffer.data(), l/downsampling, 0), "Error writing scanline");
    }
}