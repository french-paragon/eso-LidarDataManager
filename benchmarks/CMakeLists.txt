
find_package(benchmark REQUIRED)

set(PROCESSING_BLOCKS_LIST ${PROC_BLOCKS_FILES})
list(TRANSFORM PROCESSING_BLOCKS_LIST PREPEND ../)

add_executable(benchmarkProcessingBlocks benchmark_processing_blocks.cpp ${PROCESSING_BLOCKS_LIST})
target_link_libraries(benchmarkProcessingBlocks StereoVision::stevi PROJ::proj benchmark::benchmark)

add_custom_target(benchmark COMMAND benchmarkProcessingBlocks)
