FROM ubuntu:22.04

COPY docker/setup_script.bash /
RUN bash setup_script.bash

COPY CMakeLists.txt *.cpp *.h /
COPY processingBlocks /processingBlocks

RUN cmake -DCMAKE_BUILD_TYPE=Release .; make

ENTRYPOINT ["./lidarDataManager"]
