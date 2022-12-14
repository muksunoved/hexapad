find_package(Qt5 COMPONENTS Widgets SerialPort REQUIRED)

set (SOURCE
    mainwindow.cpp
    master_thread.cpp
    ComPort.cpp
    resources.qrc
    ../inc/mainwindow.h
    ../inc/master_thread.h
    )



add_executable(${PROJECT_NAME}
    main.cpp
    ${SOURCE}
)

if(CMAKE_BUILD_TYPE STREQUAL "Release")
  set_property(TARGET ${PROJECT_NAME} PROPERTY WIN32_EXECUTABLE true)
endif()

target_link_libraries(${PROJECT_NAME} Qt5::Widgets Qt5::SerialPort)


  
