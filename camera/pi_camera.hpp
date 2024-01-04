#pragma once

#include <boost/property_tree/json_parser.hpp>

#include "core/libcamera_app.hpp"
#include "v4l2_camera.hpp"

class PiCamera : public V4l2Camera
{
public:
    PiCamera(const boost::property_tree::ptree &camera_config);

    virtual std::optional<ICameraOptions> options() const;

protected:
    virtual void connect_camera(const boost::property_tree::ptree &camera_config);
    virtual ImageData block_and_receive_image();

private:
    LibcameraApp camera_;
};
