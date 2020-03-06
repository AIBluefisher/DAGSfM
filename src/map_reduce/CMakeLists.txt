set(FOLDER_NAME "map_reduce")

COLMAP_ADD_SOURCES(
    communicator.h communicator.cpp
    map_reduce_config.h 
    map_reduce_result.h
    mapper.h
    master.h master.cpp
    msgpack_adaptor.h
    reducer.h
    running_info.h
    worker.h worker.cpp
)

COLMAP_ADD_TEST(master_test master_test.cpp)
COLMAP_ADD_TEST(worker_test worker_test.cpp)
COLMAP_ADD_TEST(msgpack_adaptor_test msgpack_adaptor_test.cpp)
