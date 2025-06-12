cmake_minimum_required(VERSION 3.22)
project(ufr_gtw_telegram)

SET(VERSION 4.7.0)

INCLUDE(FindPkgConfig)
pkg_check_modules(curl REQUIRED "libcurl")
pkg_check_modules(json REQUIRED "json-c")


# SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${EXTRA_LIB_CFLAGS} -Werror -Wall -Wno-unused-function" )
include_directories(libtelebot/include)

message(${PKGS_CFLAGS})
SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${curl_CFLAGS} ${json_CFLAGS}") # -Werror -Wall -Wno-unused-function" )

add_library(ufr_gtw_telegram SHARED 
    ./src/ufr_gtw_telegram.c
    ./libtelebot/src/telebot-parser.c
    ./libtelebot/src/telebot-core.c
    ./libtelebot/src/telebot.c
    )
target_link_libraries(ufr_gtw_telegram ${curl_LDFLAGS} ${json_LDFLAGS} )
SET_TARGET_PROPERTIES(ufr_gtw_telegram PROPERTIES VERSION ${VERSION})


add_executable(example_telegram ./examples/bot.c)
target_link_libraries(example_telegram ufr ufr_gtw_telegram)
