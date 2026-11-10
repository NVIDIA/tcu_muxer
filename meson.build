project('tcu_muxer', 'c', version: '0.1', meson_version: '>=1.6.0')

thread_dep = dependency('threads')

executable(
    'tcu_muxer',
    ['tcu_com.c', 'raw_capture.c', 'rx_stall.c'],
    dependencies: thread_dep,
)

if get_option('tests').allowed()
    python = find_program('python3')
    hup_test_muxer = executable(
        'tcu_muxer_hup_test',
        ['tcu_com.c', 'raw_capture.c', 'rx_stall.c'],
        c_args: '-DUUCP_DIR="/tmp"',
        dependencies: thread_dep,
    )
    test(
        'physical-uart-hup-reopen',
        python,
        args: [files('tests/test_physical_uart_hup.py'), hup_test_muxer],
        timeout: 60,
    )
    rx_stall_test = executable(
        'tcu_muxer_rx_stall_test',
        'tests/test_rx_stall.c',
    )
    test('rx-stall-detection-and-backoff', rx_stall_test)
    stall_fixture = executable(
        'tcu_muxer_stall_fixture',
        ['tcu_com.c', 'raw_capture.c', 'rx_stall.c', 'tests/rx_stall_fixture.c'],
        c_args: '-DUUCP_DIR="/tmp"',
        link_args: '-Wl,--wrap=rx_stall_check',
        dependencies: thread_dep,
    )
    test(
        'silent-rx-stall-reopen',
        python,
        args: [files('tests/test_silent_rx_stall.py'), stall_fixture],
        timeout: 20,
    )
endif
