
set(imu_src src/serial.c src/wit_c_sdk.c)
#set(imu_include include/REG.h include/serial.h include/wit_c_sdk.h)
#add_library(libimu ${imu_src} ${imu_include})
#set(imu_include include/REG.h include/serial.h include/wit_c_sdk.h)
add_library(libimu ${imu_src})
target_include_directories(libimu PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)


