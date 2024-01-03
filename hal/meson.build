pigpio_inc = include_directories('/usr/include/', is_system: true)
pigpio_dep = declare_dependency(
    link_args: '-lpigpio',
    include_directories: pigpio_inc
)

hal_454_src += files([
    'hal.cpp',
    'motor.cpp'
])

hal_454_dep += [pigpio_dep]
