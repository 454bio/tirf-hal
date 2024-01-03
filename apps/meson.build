capture_service = executable(
    'capture-service',
    files('capture_service.cpp'),
    include_directories: include_directories('..'),
    dependencies: [rpicam_apps_dep, boost_dep, pigpio_dep, thread_dep],
    link_with: [rpicam_apps_lib, hal_454],
    install: true
)
