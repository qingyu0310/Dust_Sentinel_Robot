# 烧录器参数由 sdk_glue_user/boards/hpmicro/hpm5e31gn1/board.cmake 给出
# （adapter speed 500 + openocd-search 两条），这里不再用 app_set_runner_args 覆盖，
# 否则会重复下发 adapter speed 导致 JTAG 链识别失败。
