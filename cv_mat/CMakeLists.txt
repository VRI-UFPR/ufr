find_package(OpenCV)

include_directories(./include)

add_library(cv_mat STATIC src/cv_mat.cpp)
target_link_libraries(cv_mat ${OpenCV_LIBS})
target_include_directories(cv_mat PUBLIC "${OPENCV_INCLUDE_DIR}")

add_executable(simple examples/simple.c)
target_link_libraries(simple cv_mat)

# install(TARGETS teste2 DESTINATION examples)
