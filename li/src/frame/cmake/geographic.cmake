#包含子目录
add_subdirectory(${PROJECT_SOURCE_DIR}/third_party/GeographicLib)

include_directories(${PROJECT_SOURCE_DIR}/third_party/GeographicLib/include/)

#把库的名字合并到变量ALL_TARGET_LIBRARIES中去
#list表示数据结构为list(列表)
#APPEND表示将第二个变量添加之第一个变量末尾
list(APPEND ALL_TARGET_LIBRARIES libGeographiccc)