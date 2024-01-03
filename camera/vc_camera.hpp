#pragma once

// TODO: Prune these
#include "vendor/vcmipidemo/src/vctypes.h"
#include "vendor/vcmipidemo/src/vclib-excerpt.h"
#include "vendor/vcmipidemo/src/vcimgnet.h"
#include "vendor/vcmipidemo/src/vcmipilib.h"

#include "v4l2_camera.hpp"

class VisionComponentsCamera : public V4l2Camera
{
public:
    VisionComponentsCamera(const boost::property_tree::ptree &camera_config);

    virtual std::optional<ICameraOptions> options() const;

protected:
    virtual void connect_camera(const boost::property_tree::ptree &camera_config);
    virtual ImageData block_and_receive_image();

private:
    void set_parameters(const boost::property_tree::ptree &parameters);

    std::string camera_device_path_;

    uint32_t exposure_time_us_;

    VCMipiSenCfg sensor_;
    unsigned int buffer_index_;
};
