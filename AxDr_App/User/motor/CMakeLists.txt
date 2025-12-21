sdk_inc(.)

file(GLOB_RECURSE SOURCES
        "*.c"
)
foreach(SOURCE_FILE ${SOURCES})
    sdk_src(${SOURCE_FILE})
endforeach()

