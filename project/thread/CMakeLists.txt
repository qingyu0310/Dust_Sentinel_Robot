# 业务线程装配 — 门禁 TRD_XXX 保持原名（不加 DUST_ 前缀）
#
# 新增一个线程时：
#   1. 在 project/Kconfig 加 config TRD_XXX
#   2. 在下面照这块追加
#
# if(CONFIG_TRD_XXX)
# 	target_sources(app PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/xxx/trd_xxx.cpp)
# 	target_include_directories(app PRIVATE
# 		${CMAKE_CURRENT_SOURCE_DIR}
# 		${CMAKE_CURRENT_SOURCE_DIR}/xxx
# 	)
# endif()

if(CONFIG_TRD_CAN_TX)
	target_sources(app PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/can/trd_can_tx.cpp)
	target_include_directories(app PRIVATE
		${CMAKE_CURRENT_SOURCE_DIR}
		${CMAKE_CURRENT_SOURCE_DIR}/can
	)
endif()

if(CONFIG_TRD_CHASSIS)
	target_sources(app PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/chassis/trd_chassis.cpp)
	target_include_directories(app PRIVATE
		${CMAKE_CURRENT_SOURCE_DIR}
		${CMAKE_CURRENT_SOURCE_DIR}/chassis
	)
endif()

# 云台公共命令：pitch / yaw 任一启用就需要
if(CONFIG_TRD_YAW OR CONFIG_TRD_PITCH)
	target_sources(app PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/gimbal/gimbal.cpp)
	target_include_directories(app PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/gimbal)
endif()

if(CONFIG_TRD_YAW)
	target_sources(app PRIVATE
		${CMAKE_CURRENT_SOURCE_DIR}/gimbal/yaw/trd_yaw.cpp
		${CMAKE_CURRENT_SOURCE_DIR}/gimbal/yaw/trd_yaw_coop.cpp
	)
	target_include_directories(app PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/gimbal/yaw)
endif()

if(CONFIG_TRD_PITCH)
	target_sources(app PRIVATE
		${CMAKE_CURRENT_SOURCE_DIR}/gimbal/pitch/trd_pitch.cpp
	)
	target_include_directories(app PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/gimbal/pitch)
endif()

if(CONFIG_TRD_IMU)
	target_sources(app PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/imu/trd_imu.cpp)
	target_include_directories(app PRIVATE
		${CMAKE_CURRENT_SOURCE_DIR}
		${CMAKE_CURRENT_SOURCE_DIR}/imu
	)
endif()

if(CONFIG_TRD_GPIO)
	target_sources(app PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/gpio/trd_gpio.cpp)
	target_include_directories(app PRIVATE
		${CMAKE_CURRENT_SOURCE_DIR}
		${CMAKE_CURRENT_SOURCE_DIR}/gpio
	)
endif()

if(CONFIG_TRD_REMOTE)
	target_sources(app PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/remote/trd_remote.cpp)
	target_include_directories(app PRIVATE
		${CMAKE_CURRENT_SOURCE_DIR}
		${CMAKE_CURRENT_SOURCE_DIR}/remote
	)
endif()

if(CONFIG_TRD_TEST)
	target_sources(app PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/test/trd_test.cpp)
	target_include_directories(app PRIVATE
		${CMAKE_CURRENT_SOURCE_DIR}
		${CMAKE_CURRENT_SOURCE_DIR}/test
		${CMAKE_CURRENT_SOURCE_DIR}/pitch
	)
endif()
