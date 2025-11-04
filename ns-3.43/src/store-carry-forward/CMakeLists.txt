build_lib(
  LIBNAME store-carry-forward
  SOURCE_FILES
    model/store-carry-forward.cc
  HEADER_FILES
    model/store-carry-forward.h
  LIBRARIES_TO_LINK
    ${libcore}
    ${libnetwork}
    ${libmobility}
    ${libapplications}
)
