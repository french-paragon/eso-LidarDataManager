
find_package(GTest REQUIRED)

set(PROCESSING_BLOCKS_LIST ${PROC_BLOCKS_FILES})
list(TRANSFORM PROCESSING_BLOCKS_LIST PREPEND ../)

add_executable(testProcessingBlocks test_processing_blocks.cpp ${PROCESSING_BLOCKS_LIST})
target_link_libraries(testProcessingBlocks StereoVision::stevi PROJ::proj GTest::gtest GTest::gtest_main)

add_test(NAME TestProcessingBlocks COMMAND testProcessingBlocks)

add_custom_target(test-details COMMAND testProcessingBlocks)
