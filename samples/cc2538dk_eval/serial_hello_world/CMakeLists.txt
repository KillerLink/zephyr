cmake_minimum_required(VERSION 3.12)
set(BOARD cc2538dk)
set(CONF_FILE "../confs/base.conf")
list(APPEND CONF_FILE "../confs/serial.conf")

include ($ENV{ZEPHYR_BASE}/cmake/app/boilerplate.cmake NO_POLICY_SCOPE)
project(cc2538_net_test_udp_client)

target_sources(app PRIVATE src/main.c)
