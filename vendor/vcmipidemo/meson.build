rt = cxx.find_library(
    'rt',
    required: true
)

vclib_excerpt = library(
    'vclib-excerpt',
    files('src/vclib-excerpt.c')
)

vcimgnet = library(
    'vcimgnet',
    files('src/vcimgnet.c'),
    link_with: vclib_excerpt,
    dependencies: rt
)

vcimgnetsrv = executable(
    'vcimgnetsrv',
    files('src/vcimgnetsrv.c'),
    link_with: vclib_excerpt,
    dependencies: rt
)

vcmipilib = library(
    'vcmipilib',
    files('src/vcmipidemo.c'),
    link_with: [vclib_excerpt, vcimgnet]
)

vcmipidemo = executable(
    'vcmipidemo',
    link_with: [vcmipilib, vclib_excerpt, vcimgnet]
)
