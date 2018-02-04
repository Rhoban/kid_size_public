set(SOURCES
    ImageSequence.cpp
)

if (ENABLE_FIT_OPTIMIZATIONS)
    add_definitions(-DENABLE_FIT_OPTIMIZATIONS)
    SET (SOURCES
        ${SOURCES}
        yuv_convert.s
        )
endif ()
