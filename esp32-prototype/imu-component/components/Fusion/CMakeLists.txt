file(GLOB_RECURSE files "*.c")

# add_library(Fusion ${files})

idf_component_register(SRCS ${files}
                    INCLUDE_DIRS ".")


# if(UNIX AND NOT APPLE)
#     target_link_libraries(Fusion m) # link math library for Linux
# endif()