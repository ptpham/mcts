set (CMAKE_CXX_FLAGS "-g -std=c++11 -stdlib=libc++ -W -Wextra -pedantic -Wall -Wno-deprecated")
set (CMAKE_CXX_COMPILER "clang++")
add_executable (mcts mcts.cpp)
