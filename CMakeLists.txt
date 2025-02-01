# CMakeList.txt : projet CMake pour seatalk, incluez la source et définissez
# la logique spécifique au projet ici.
#
cmake_minimum_required (VERSION 3.8)

# Activez Rechargement à chaud pour les compilateurs MSVC si cela est pris en charge.
if (POLICY CMP0141)
  cmake_policy(SET CMP0141 NEW)
  set(CMAKE_MSVC_DEBUG_INFORMATION_FORMAT "$<IF:$<AND:$<C_COMPILER_ID:MSVC>,$<CXX_COMPILER_ID:MSVC>>,$<$<CONFIG:Debug,RelWithDebInfo>:EditAndContinue>,$<$<CONFIG:Debug,RelWithDebInfo>:ProgramDatabase>>")
endif()

project ("seatalk")

add_executable (test 
test/seatalk_test.c 
boat_sensor.c
boat_status.c
seatalk_command.c
seatalk_datagram.c
seatalk_protocol.c
seatalk_transport_layer.c
test/assertions.c
test/boat_sensor_test.c
test/boat_status_test.c
test/seatalk_command_test.c
test/seatalk_datagram_test.c
test/seatalk_hardware_layer.c
test/seatalk_protocol_test.c
test/seatalk_test.c
test/seatalk_transport_layer_test.c
test/timeout.c  )
target_compile_definitions(test PUBLIC TESTING)
target_compile_definitions(test PUBLIC _CRT_SECURE_NO_WARNINGS)
if (CMAKE_VERSION VERSION_GREATER 3.12)
  set_property(TARGET test PROPERTY CXX_STANDARD 20)
endif()

add_executable (seatalk-y
boat_sensor.c
boat_status.c
seatalk_command.c
seatalk_datagram.c
seatalk_protocol.c
seatalk_transport_layer.c
linux/seatalk_module.c
linux/seatalk_hardware_layer.c
  )
target_compile_definitions(seatalk-y PUBLIC _CRT_SECURE_NO_WARNINGS)
if (CMAKE_VERSION VERSION_GREATER 3.12)
  set_property(TARGET seatalk-y PROPERTY CXX_STANDARD 20)
endif()



# TODO: Ajoutez des tests et installez des cibles si nécessaire.
