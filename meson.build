project(
    '454-hal', 'c', 'cpp',
    version: '6.5.6',
    default_options: [
        'werror=true',
        'warning_level=3',
        'cpp_std=c++17',
        'buildtype=release',
    ]
)

rpicam_apps = subproject('rpicam-apps')

# cpp_arguments = [
#     '-pedantic',
#     '-Wno-unused-parameter',
#     '-faligned-new',
#     '-DNOMINMAX'
# ]

exe = executable('454_hal', '454_hal.c',
    install: true)

test('basic', exe)
