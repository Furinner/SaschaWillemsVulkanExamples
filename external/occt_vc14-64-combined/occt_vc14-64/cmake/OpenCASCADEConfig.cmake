#-----------------------------------------------------------------------------
#
# OpenCASCADEConfig.cmake - OpenCASCADE CMake configuration file for external projects.
#
# This file is configured by OpenCASCADE.
#

if(OpenCASCADE_ALREADY_INCLUDED)
  return()
endif()
set(OpenCASCADE_ALREADY_INCLUDED 1)

# The OpenCASCADE version number
set (OpenCASCADE_MAJOR_VERSION       "7")
set (OpenCASCADE_MINOR_VERSION       "9")
set (OpenCASCADE_MAINTENANCE_VERSION "1")
set (OpenCASCADE_DEVELOPMENT_VERSION "")

# Compute the installation prefix from this OpenCASCADEConfig.cmake file 
# location, by going up one level + one level if "cmake" + one level if "lib".
# This is made to support different locations of CMake files:
# - in UNIX style: $INSTALL_DIR/lib/cmake/opencascade-<version>
# - in Windows style: $INSTALL_DIR/cmake
# - in Android style: $INSTALL_DIR/libs/$CMAKE_ANDROID_ARCH_ABI/cmake/opencascade-<version>
get_filename_component (OpenCASCADE_INSTALL_PREFIX "${CMAKE_CURRENT_LIST_FILE}" PATH)
get_filename_component (OpenCASCADE_INSTALL_PREFIX "${OpenCASCADE_INSTALL_PREFIX}" PATH)
if (OpenCASCADE_INSTALL_PREFIX MATCHES "/cmake$")
  get_filename_component (OpenCASCADE_INSTALL_PREFIX "${OpenCASCADE_INSTALL_PREFIX}" PATH)
endif()
if (OpenCASCADE_INSTALL_PREFIX MATCHES "/lib(32|64)?$")
  get_filename_component (OpenCASCADE_INSTALL_PREFIX "${OpenCASCADE_INSTALL_PREFIX}" PATH)
endif()
if (OpenCASCADE_INSTALL_PREFIX MATCHES "/libs/${CMAKE_ANDROID_ARCH_ABI}$")
  get_filename_component (OpenCASCADE_INSTALL_PREFIX "${OpenCASCADE_INSTALL_PREFIX}" PATH)
  get_filename_component (OpenCASCADE_INSTALL_PREFIX "${OpenCASCADE_INSTALL_PREFIX}" PATH)
endif()

# Set OpenCASCADE paths to headers, binaries, libraries, resources, tests, samples, data
set (OpenCASCADE_BINARY_DIR   "${OpenCASCADE_INSTALL_PREFIX}/win64/vc14/bin")
set (OpenCASCADE_LIBRARY_DIR  "${OpenCASCADE_INSTALL_PREFIX}/win64/vc14/lib")
set (OpenCASCADE_SCRIPT_DIR   "${OpenCASCADE_INSTALL_PREFIX}/.")
set (OpenCASCADE_INCLUDE_DIR  "${OpenCASCADE_INSTALL_PREFIX}/inc")
set (OpenCASCADE_RESOURCE_DIR "${OpenCASCADE_INSTALL_PREFIX}/src")

# The C and C++ flags added by OpenCASCADE to the cmake-configured flags.
set (OpenCASCADE_C_FLAGS      "  /W3   /fp:precise")
set (OpenCASCADE_CXX_FLAGS    "  /W4 /GR /EHa /fp:precise /fp:precise /wd26812 /MP")
set (OpenCASCADE_LINKER_FLAGS   "/machine:x64")

# List of available OpenCASCADE modules.
set (OpenCASCADE_MODULES FoundationClasses;ModelingData;ModelingAlgorithms;Visualization;ApplicationFramework;DataExchange;DETools;Draw)

# List of available OpenCASCADE libraries for each module
set (OpenCASCADE_FoundationClasses_LIBRARIES TKernel;TKMath)
set (OpenCASCADE_ModelingData_LIBRARIES TKG2d;TKG3d;TKGeomBase;TKBRep)
set (OpenCASCADE_ModelingAlgorithms_LIBRARIES TKGeomAlgo;TKTopAlgo;TKPrim;TKBO;TKShHealing;TKBool;TKHLR;TKFillet;TKOffset;TKFeat;TKMesh;TKXMesh)
set (OpenCASCADE_Visualization_LIBRARIES TKService;TKV3d;TKOpenGl;TKOpenGles;TKMeshVS;TKIVtk;TKD3DHost)
set (OpenCASCADE_ApplicationFramework_LIBRARIES TKCDF;TKLCAF;TKCAF;TKBinL;TKXmlL;TKBin;TKXml;TKStdL;TKStd;TKTObj;TKBinTObj;TKXmlTObj;TKVCAF)
set (OpenCASCADE_DataExchange_LIBRARIES TKDE;TKXSBase;TKDESTEP;TKXCAF;TKDEIGES;TKDESTL;TKDEVRML;TKRWMesh;TKDECascade;TKBinXCAF;TKXmlXCAF;TKDEOBJ;TKDEGLTF;TKDEPLY)
set (OpenCASCADE_DETools_LIBRARIES TKExpress)
set (OpenCASCADE_Draw_LIBRARIES TKDraw;TKTopTest;TKOpenGlTest;TKViewerTest;TKOpenGlesTest;TKD3DHostTest;TKXSDRAW;TKDCAF;TKXDEDRAW;TKTObjDRAW;TKQADraw;TKIVtkDraw;TKXSDRAWDE;TKXSDRAWGLTF;TKXSDRAWIGES;TKXSDRAWOBJ;TKXSDRAWPLY;TKXSDRAWSTEP;TKXSDRAWSTL;TKXSDRAWVRML)

# List of available OpenCASCADE libraries.
set (OpenCASCADE_LIBRARIES TKernel;TKMath;TKG2d;TKG3d;TKGeomBase;TKBRep;TKGeomAlgo;TKTopAlgo;TKPrim;TKBO;TKShHealing;TKBool;TKHLR;TKFillet;TKOffset;TKFeat;TKMesh;TKXMesh;TKService;TKV3d;TKOpenGl;TKOpenGles;TKMeshVS;TKIVtk;TKD3DHost;TKCDF;TKLCAF;TKCAF;TKBinL;TKXmlL;TKBin;TKXml;TKStdL;TKStd;TKTObj;TKBinTObj;TKXmlTObj;TKVCAF;TKDE;TKXSBase;TKDESTEP;TKXCAF;TKDEIGES;TKDESTL;TKDEVRML;TKRWMesh;TKDECascade;TKBinXCAF;TKXmlXCAF;TKDEOBJ;TKDEGLTF;TKDEPLY;TKExpress;TKDraw;TKTopTest;TKOpenGlTest;TKViewerTest;TKOpenGlesTest;TKD3DHostTest;TKXSDRAW;TKDCAF;TKXDEDRAW;TKTObjDRAW;TKQADraw;TKIVtkDraw;TKXSDRAWDE;TKXSDRAWGLTF;TKXSDRAWIGES;TKXSDRAWOBJ;TKXSDRAWPLY;TKXSDRAWSTEP;TKXSDRAWSTL;TKXSDRAWVRML)

# OpenCASCADE global configuration options.
set (OpenCASCADE_COMPILER          "vc14")
set (OpenCASCADE_BUILD_WITH_DEBUG  FALSE)
set (OpenCASCADE_BUILD_SHARED_LIBS ON)


# Use of third-party libraries.
set (OpenCASCADE_WITH_TCL       ON)
set (OpenCASCADE_WITH_FREETYPE  TRUE)
set (OpenCASCADE_WITH_FREEIMAGE TRUE)
set (OpenCASCADE_WITH_TBB       TRUE)
set (OpenCASCADE_WITH_VTK       TRUE)
set (OpenCASCADE_WITH_FFMPEG    TRUE)
set (OpenCASCADE_WITH_GLES2     TRUE)
set (OpenCASCADE_WITH_D3D       TRUE)


# Import OpenCASCADE compile definitions, C and C++ flags for each installed configuration.
file(GLOB CONFIG_FILES "${CMAKE_CURRENT_LIST_DIR}/OpenCASCADECompileDefinitionsAndFlags-*.cmake")
foreach(f ${CONFIG_FILES})
  include(${f})
endforeach()

if (NOT OpenCASCADE_FIND_COMPONENTS)
  set (OpenCASCADE_FIND_COMPONENTS ${OpenCASCADE_MODULES})
endif ()

# Import OpenCASCADE targets.
foreach(_comp ${OpenCASCADE_FIND_COMPONENTS})
  if (NOT ";${OpenCASCADE_MODULES};" MATCHES "${_comp}")
    set(OpenCASCADE_FOUND False)
    set(OpenCASCADE_NOTFOUND_MESSAGE "Specified unsupported component: ${_comp}")
    if (NOT OpenCASCADE_FIND_QUIETLY)
      message (ERROR ": ${OpenCASCADE_NOTFOUND_MESSAGE}")
    endif()
  else()
    include("${CMAKE_CURRENT_LIST_DIR}/OpenCASCADE${_comp}Targets.cmake")
  endif()
endforeach()
