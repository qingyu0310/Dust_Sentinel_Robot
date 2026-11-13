# USB DCD QTD 深队列 8→2（CDC 用不到 8 深；覆盖 sdk_env hpm_soc_feature.h 的 #ifndef 宏，QTD 256→64 个，省 6KB）
add_compile_definitions(USB_SOC_DCD_QTD_COUNT_EACH_ENDPOINT=2)
