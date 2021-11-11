cmake_minimum_required(VERSION 3.5)

project(Doosan LANGUAGES CXX C)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_C_COMPILER "g++")

# open62541 v1.2 must be installed!
# If in custom path, then use -DCMAKE_PREFIX_PATH=/home/user/installcmake_minimum_required(VERSION 3.2)
find_package(open62541 REQUIRED COMPONENTS FullNamespace)
find_package(Poco REQUIRED COMPONENTS Foundation Net Util)
find_package(Boost REQUIRED COMPONENTS thread)
find_package(Eigen3 REQUIRED)

# SAMYLibPlugin.a includes
include_directories("${PROJECT_SOURCE_DIR}/ThirdParty/SAMY/Communication/include"
                    "${PROJECT_SOURCE_DIR}/ThirdParty/SAMY/Plugin/include"
                    "${PROJECT_SOURCE_DIR}/ThirdParty/SAMY/Helpers/include"
                    "${PROJECT_SOURCE_DIR}/ThirdParty/SAMY/OPCUA_CRCL/pubsub"
                    "${PROJECT_SOURCE_DIR}/ThirdParty/SAMY/OPCUA_CRCL/deps"
                    "${PROJECT_SOURCE_DIR}/ThirdParty/SAMY/OPCUA_CRCL/include"
                    "${PROJECT_SOURCE_DIR}/ThirdParty/SAMY/OPCUA_CRCL/include/open62541/plugin"
                    "${PROJECT_SOURCE_DIR}/ThirdParty/SAMY/OPCUA_CRCL/plugins/"
#                    "${PROJECT_SOURCE_DIR}/build/src_generated")
                    )

# The generated files are: "namespace_${NAMESPACE_NAME}_... .c/.h"
 set(NAMESPACE_NAME_CRCL "crcl")
# # The compiled namespace is: "${NAMESPACE_PREFIX}-ns-${NAMESPACE_NAME}"
 set(NAMESPACE_PREFIX "pref")

 set(GENERATE_OUTPUT_DIR "${CMAKE_BINARY_DIR}/src_generated/")
 include_directories("${GENERATE_OUTPUT_DIR}")
 file(MAKE_DIRECTORY "${GENERATE_OUTPUT_DIR}")

 ua_generate_nodeset_and_datatypes(
         INTERNAL
         NAME "${NAMESPACE_NAME_CRCL}"
         FILE_NS "${CMAKE_CURRENT_LIST_DIR}/ThirdParty/SAMY/OPCUA_CRCL/model/Published/CRCL.NodeSet2.xml"
         TARGET_PREFIX "${NAMESPACE_PREFIX}"
         FILE_CSV "${CMAKE_CURRENT_LIST_DIR}/ThirdParty/SAMY/OPCUA_CRCL/model/Published/CRCL.csv"
         FILE_BSD "${CMAKE_CURRENT_LIST_DIR}/ThirdParty/SAMY/OPCUA_CRCL/model/Published/CRCL.Types.bsd"
         OUTPUT_DIR "${GENERATE_OUTPUT_DIR}"
#     # This namespace index must match the order in which you are adding the nodeset in the source code
         NAMESPACE_MAP 2:https://crcl.org
)

# You also need to build the SAMYPluginLibrary, otherwise it is very unlikely that will work if compiled in a different system (different ABIs, etc.)
add_library(SAMYPluginLib STATIC
            ${UA_NODESET_CRCL_SOURCES}
            ${UA_TYPES_CRCL_SOURCES}
            #"./src/Communication/samy_robot.c"
            "./ThirdParty/SAMY/Plugin/plugin.cpp"
            "./ThirdParty/SAMY/Helpers/helpers.cpp"
            )

# Make sure the nodeset compiler is executed before compiling the main file (aka generate files)
add_dependencies(SAMYPluginLib
                 ${NAMESPACE_PREFIX}-ns-${NAMESPACE_NAME_CRCL}
                 )

include_directories("${PROJECT_SOURCE_DIR}/src/Robot/include"
                    "${PROJECT_SOURCE_DIR}/ThirdParty/Doosan/include")
include_directories(${EIGEN3_INCLUDE_DIRS})

#link_directories("${PROJECT_SOURCE_DIR}/ThirdParty/SAMY/"
#                   "${PROJECT_SOURCE_DIR}/ThirdParty/Doosan/lib")

target_link_libraries(SAMYPluginLib PRIVATE ${Boost_LIBRARIES} open62541::open62541)



# Build Plugin Specific part
link_directories("${PROJECT_SOURCE_DIR}/ThirdParty/Doosan/lib")

add_executable(${PROJECT_NAME} main.cpp
                      "src/Robot/robot.cpp")

# Make sure the nodeset compiler is executed before compiling the main file (aka generate files)
add_dependencies(${PROJECT_NAME}
                 ${NAMESPACE_PREFIX}-ns-${NAMESPACE_NAME_CRCL}
                 )

target_link_libraries(${PROJECT_NAME} PRIVATE SAMYPluginLib DRFL Poco::Foundation Poco::Net ${Boost_LIBRARIES} open62541::open62541)
