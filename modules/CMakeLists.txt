# 用户层 modules 装配 — 门禁 DUST_MOD_DEV_XXX
# 本文件由 project/CMakeLists.txt 的 add_subdirectory(${APP_MODULES}) 引入
#
# 新增内容时照这块追加：
#
# if(CONFIG_DUST_MOD_DEV_XXX)
# 	target_sources(app PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/xxx/xxx.cpp)
# 	target_include_directories(app PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/xxx)
# endif()

if(CONFIG_DUST_MOD_DEV_REMOTE)
	target_sources(app PRIVATE
		${CMAKE_CURRENT_SOURCE_DIR}/remotes/remote_map_dr16.cpp
		${CMAKE_CURRENT_SOURCE_DIR}/remotes/remote_map_sbus.cpp
	)
	target_include_directories(app PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/remotes)
endif()
