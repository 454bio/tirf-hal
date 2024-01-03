libcamera_dep = dependency('libcamera', required : true)

rpicam_apps_lib = rpicam_apps.get_variable('libcamera_app')
rpicam_apps_dep = declare_dependency(
    include_directories: include_directories([
        '../subprojects/rpicam-apps',
        '/usr/include/libcamera'
    ]),
    link_with: rpicam_apps_lib
)

tiff_dep = dependency('libtiff-4', required : true)
tiffxx_dep = declare_dependency(
    link_args: '-ltiffxx'
)

hal_454_src += files([
    'image_buffer.cpp',
    'pi_camera.cpp',
    'pylablib_camera.cpp',
    'tiff.cpp',
    'v4l2_camera.cpp',
    'vc_camera.cpp'
])

hal_454_dep += [libcamera_dep, rpicam_apps_dep, tiff_dep, tiffxx_dep]
