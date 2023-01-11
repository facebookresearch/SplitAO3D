set(LIBS research_framework rlr_streaming glog gflags)

set(SHARED_SRC KeyPress.cpp KeyPress.h Flags.h)

# Sample that demonstrates sever and client that both use in memory streaming
# both running within the same process.
add_executable(
  InProcessStreaming InProcessStreamingMain.cpp SimpleClient.cpp
                     SimpleServer.cpp SimpleClient.h SimpleServer.h ${SHARED_SRC})
target_link_libraries(InProcessStreaming PRIVATE ${LIBS})

# Sample that demonstrates server and client that use WebRTC streaming both
# running as separate processes.
add_executable(WebrtcServer WebrtcServerMain.cpp SimpleServer.cpp
                            SimpleServer.h ${SHARED_SRC})
target_link_libraries(WebrtcServer PRIVATE ${LIBS})
add_executable(WebrtcClient WebrtcClientMain.cpp SimpleClient.cpp
                            SimpleClient.h ${SHARED_SRC})
target_link_libraries(WebrtcClient PRIVATE ${LIBS})

# Disable warning C26451: Arithmetic overflow
target_compile_options(WebrtcServer PRIVATE $<$<CXX_COMPILER_ID:MSVC>: /wd26451
                                            >)
