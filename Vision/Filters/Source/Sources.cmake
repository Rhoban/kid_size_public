set (SOURCES
  Source.cpp
  SourceFactory.cpp
  SourceLogs.cpp
  SourceOpenCV.cpp
  SourceVideoProtobuf.cpp
)


if (KID_SIZE_USES_FLYCAPTURE)
  set(SOURCES "${SOURCES}" 
    SourcePtGrey.cpp
  )
endif(KID_SIZE_USES_FLYCAPTURE)

if (KID_SIZE_USES_IDS)
  set(SOURCES "${SOURCES}" 
    SourceIDS.cpp
  )
endif(KID_SIZE_USES_IDS)
