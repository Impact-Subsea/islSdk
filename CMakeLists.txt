cmake_minimum_required(VERSION 3.0.0)

project(islSdk LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(LIBRARY_OUTPUT_PATH  ${CMAKE_BINARY_DIR}/lib)

if (X32_BIT)
    add_definitions(-DSYSTEM_32BIT)
    message(STATUS "SDK build config 32 bit")  
else()
    add_definitions(-DSYSTEM_64BIT)
    message(STATUS "SDK build config 64 bit") 
endif()

if(WIN32)
    set(PLATFORM_DIR windows)
    add_definitions(-DOS_WINDOWS)
    message(STATUS "SDK build config Windows")
elseif(UNIX)
    set(PLATFORM_DIR unix)
    add_definitions(-DOS_UNIX)
    message(STATUS "SDK build config Unix")
endif()

if(${CMAKE_BUILD_TYPE} MATCHES Debug)
    add_definitions(-DDEBUG_ON)
    message(STATUS "SDK build config Debug info on")
elseif(${CMAKE_BUILD_TYPE} MATCHES Release)
    message(STATUS "SDK build config Release mode")
endif()

set(HEADERS
    src/comms/discovery/autoDiscovery.h
    src/comms/discovery/islDeviceDiscovery.h
    src/comms/discovery/nmeaDiscovery.h
    src/comms/ports/netPort.h
    src/comms/ports/solPort.h
    src/comms/ports/sysPort.h
    src/comms/ports/uartPort.h
    src/comms/protocols/cobs.h
    src/comms/protocols/codec.h
    src/comms/protocols/nmea.h
    src/comms/connectionMeta.h
    src/comms/islHdlc.h
    src/comms/sysPortMgr.h
    src/comms/protocolDebugger.h
    src/devices/ahrs.h
    src/devices/device.h
    src/devices/deviceMgr.h
    src/devices/isa500.h
    src/devices/isd4000.h
    src/devices/ism3d.h
    src/devices/sonar.h
    src/files/bmpFile.h
    src/files/logFile.h
    src/files/xmlFile.h
    src/helpers/palette.h
    src/helpers/sonarImage.h
    src/helpers/sonarDataStore.h
    src/logging/loggingDevice.h
    src/logging/logPlayer.h
    src/logging/logReader.h
    src/logging/logWriter.h
    src/maths/maths.h
    src/maths/matrix3x3.h
    src/maths/quaternion.h
    src/maths/vector3.h
    src/nmeaDevices/gpsDevice.h
    src/nmeaDevices/nmeaDevice.h
    src/nmeaDevices/nmeaDeviceMgr.h
    src/platform/debug.h
    src/platform/file.h
    src/platform/mem.h
    src/platform/timeUtils.h
    src/platform/netSocket.h
    src/platform/uart.h
    src/platform/${PLATFORM_DIR}/netSocket.h
    src/platform/${PLATFORM_DIR}/uart.h
    src/types/queue.h
    src/types/sdkTypes.h
    src/types/sigSlot.h
    src/utils/base64.h
    src/utils/crc.h
    src/utils/stringUtils.h
    src/utils/utils.h
    src/utils/xmlSettings.h
    src/sdk.h
)

set(SOURCES
    src/comms/discovery/islDeviceDiscovery.cpp
    src/comms/discovery/nmeaDiscovery.cpp
    src/comms/ports/netPort.cpp
    src/comms/ports/solPort.cpp
    src/comms/ports/sysPort.cpp
    src/comms/ports/uartPort.cpp
    src/comms/protocols/cobs.cpp
    src/comms/protocols/codec.cpp
    src/comms/protocols/nmea.cpp
    src/comms/islHdlc.cpp
    src/comms/sysPortMgr.cpp
    src/comms/protocolDebugger.cpp
    src/devices/ahrs.cpp
    src/devices/device.cpp
    src/devices/deviceMgr.cpp
    src/devices/isa500.cpp
    src/devices/isd4000.cpp
    src/devices/ism3d.cpp
    src/devices/sonar.cpp
    src/files/bmpFile.cpp
    src/files/logFile.cpp
    src/files/xmlFile.cpp
    src/helpers/palette.cpp
    src/helpers/sonarImage.cpp
    src/helpers/sonarDataStore.cpp
    src/logging/loggingDevice.cpp
    src/logging/logPlayer.cpp
    src/logging/logReader.cpp
    src/logging/logWriter.cpp
    src/maths/matrix3x3.cpp
    src/maths/quaternion.cpp
    src/maths/vector3.cpp
    src/nmeaDevices/gpsDevice.cpp
    src/nmeaDevices/nmeaDevice.cpp
    src/nmeaDevices/nmeaDeviceMgr.cpp
    src/platform/debug.cpp
    src/platform/file.cpp
    src/platform/mem.cpp
    src/platform/timeUtils.cpp
    src/platform/${PLATFORM_DIR}/netSocket.cpp
    src/platform/${PLATFORM_DIR}/uart.cpp
    src/types/queue.cpp
    src/utils/base64.cpp
    src/utils/crc.cpp
    src/utils/stringUtils.cpp
    src/utils/utils.cpp
    src/utils/xmlSettings.cpp
    src/sdk.cpp
)

add_library (${PROJECT_NAME} STATIC ${SOURCES} ${HEADERS})
target_include_directories(${PROJECT_NAME} PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})