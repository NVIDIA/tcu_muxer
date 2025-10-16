project('tcu_muxer', 'c', version: '0.1', meson_version: '>=1.6.0')

thread_dep = dependency('threads')

executable('tcu_muxer', 'tcu_com.c', dependencies: thread_dep)

if get_option('tests').allowed()
    ut = executable(
        'tcu_muxer_test',
        'tcu_com.c',
        dependencies: thread_dep,
        cpp_args: '-DUNIT_TEST',
    )

    test('tcu_muxer_test', ut)
endif
