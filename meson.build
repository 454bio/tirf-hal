project(
    '454-hal', 'c', 'cpp',
    meson_version : '>= 0.64.0',
    version: '7',
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

boost_dep = dependency('boost', modules : ['program_options', 'thread'], required : true)
thread_dep = dependency('threads', required : true)

hal_454_src = []
hal_454_dep = [boost_dep, thread_dep]

subdir('camera')
subdir('gui')
subdir('hal')
subdir('protocol')

add_project_arguments(cpp_arguments, language : 'cpp')
add_project_arguments(c_arguments, language : 'c')

subdir('vendor')

hal_454 = library(
    'hal_454',
    hal_454_src,
    soversion: meson.project_version(),
    include_directories: include_directories('.'),
    install: true,
    name_prefix: '',
    dependencies: hal_454_dep,
    link_with: vcmipilib
)

subdir('apps')
