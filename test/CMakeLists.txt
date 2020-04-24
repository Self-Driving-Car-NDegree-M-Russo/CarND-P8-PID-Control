
add_executable (pidTest testPID.cpp ../src/PID.cpp ../src/PID.h)

target_link_libraries(pidTest ${Boost_LIBRARIES})
