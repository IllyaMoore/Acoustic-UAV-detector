# Hardware-free DSP. Built as an ESP-IDF component for the firmware and as a
# plain static library by test/host/Makefile.
idf_component_register(
    SRCS "src/fft.cpp" "src/gcc_phat.cpp" "src/doa.cpp" "src/pipeline.cpp"
    INCLUDE_DIRS "include"
)
target_compile_options(${COMPONENT_LIB} PRIVATE -O2 -Wall -Wextra)
