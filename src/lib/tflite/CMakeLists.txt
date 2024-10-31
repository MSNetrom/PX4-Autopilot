############################################################################
#
#   Copyright (c) 2024 PX4 Development Team. All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in
#    the documentation and/or other materials provided with the
#    distribution.
# 3. Neither the name PX4 nor the names of its contributors may be
#    used to endorse or promote products derived from this software
#    without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
# OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
# AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
############################################################################

include_directories(${CMAKE_SOURCE_DIR}/src/lib/tflite/fixedpoint)
include_directories(${CMAKE_SOURCE_DIR}/src/lib/tflite/tensorflow)
include_directories(${CMAKE_SOURCE_DIR}/src/lib/tflite/flatbuffers)
include_directories(${CMAKE_SOURCE_DIR}/src/lib/tflite/signal)
include_directories(${CMAKE_SOURCE_DIR}/src/lib/tflite/internal)


# Define TFLite as a regular static library
add_library(tflite STATIC ${CMAKE_SOURCE_DIR}/src/lib/tflite/libtensorflow-microlite.a tflite_dummy.cpp)


# Specify include directories for dependent modules
target_include_directories(tflite PUBLIC
    ${CMAKE_SOURCE_DIR}/src/lib/tflite/fixedpoint
    ${CMAKE_SOURCE_DIR}/src/lib/tflite/tensorflow
    ${CMAKE_SOURCE_DIR}/src/lib/tflite/flatbuffers
    ${CMAKE_SOURCE_DIR}/src/lib/tflite/signal
    ${CMAKE_SOURCE_DIR}/src/lib/tflite/internal
    ${CMAKE_SOURCE_DIR}/src/lib/tflite/third_party
)
