set (SOURCES
  SourceFactory.cpp
  SourceLogs.cpp
)


if (KID_SIZE_USES_FLYCAPTURE)
  set(SOURCES "${SOURCES} SourcePtGrey.cpp")
endif(KID_SIZE_USES_FLYCAPTURE)
