project('tcu_muxer', 'c', version: '0.1', meson_version: '>=1.6.0')

thread_dep = dependency('threads')

executable(
    'tcu_muxer',
    ['tcu_com.c', 'raw_capture.c'],
    dependencies: thread_dep,
)

if get_option('tests').allowed()
    python = find_program('python3')
    hup_test_muxer = executable(
        'tcu_muxer_hup_test',
        ['tcu_com.c', 'raw_capture.c'],
        c_args: '-DUUCP_DIR="/tmp"',
        dependencies: thread_dep,
    )
    test(
        'physical-uart-hup-reopen',
        python,
        args: [files('tests/test_physical_uart_hup.py'), hup_test_muxer],
        timeout: 60,
    )
endif
