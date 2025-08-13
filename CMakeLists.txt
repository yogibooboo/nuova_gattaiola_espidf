cmake_minimum_required(VERSION 3.16.0)
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
idf_component_set_dependencies(main PRIVATE esp_websocket_server)
project(nuova_gattaiola_espidf)
