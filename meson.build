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

cpp_arguments = [
    '-pedantic',
    '-Wno-unused-parameter',
    '-faligned-new',
    '-DNOMINMAX'
]

# Needed for file sizes > 32-bits.
cpp_arguments += '-D_FILE_OFFSET_BITS=64'

cxx = meson.get_compiler('cpp')
cpu = host_machine.cpu()

if cxx.get_id() == 'gcc'
    cpp_arguments += '-Wno-psabi'
endif

if cpu == 'aarch64' or neon == 'arm64'
    cpp_arguments += '-ftree-vectorize'
endif

c_arguments = ['-Wno-implicit-fallthrough', '-Wno-unused-parameter']

add_project_arguments(cpp_arguments, language : 'cpp')
add_project_arguments(c_arguments, language : 'c')

subdir('vendor')

exe = executable('454_hal', '454_hal.c',
    install: true)

test('basic', exe)
