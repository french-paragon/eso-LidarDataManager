FROM ubuntu:24.04

COPY docker/setup_script.bash /
RUN bash setup_script.bash

COPY CMakeLists.txt *.cpp *.h /
COPY processingBlocks /processingBlocks
COPY tests /tests
COPY benchmarks /benchmarks

RUN cmake -DCMAKE_BUILD_TYPE=Release -DbuildForContainer=ON .; make

ENTRYPOINT ["./lidarDataManager"]
