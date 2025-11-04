# Copyright (c) 2016-2024, Sascha Willems
# SPDX-License-Identifier: MIT

# Function for building single example
function(buildExample EXAMPLE_NAME)
	SET(EXAMPLE_FOLDER ${CMAKE_CURRENT_SOURCE_DIR}/${EXAMPLE_NAME})
	message(STATUS "Generating project file for example in ${EXAMPLE_FOLDER}")
	# Main
	file(GLOB SOURCE *.cpp ${BASE_HEADERS} ${EXAMPLE_FOLDER}/*.cpp)
	SET(MAIN_CPP ${EXAMPLE_FOLDER}/${EXAMPLE_NAME}.cpp)
	if(EXISTS ${EXAMPLE_FOLDER}/main.cpp)
		SET(MAIN_CPP ${EXAMPLE_FOLDER}/main.cpp)
	ENDIF()
	if(EXISTS ${EXAMPLE_FOLDER}/${EXAMPLE_NAME}.h)
		SET(MAIN_HEADER ${EXAMPLE_FOLDER}/${EXAMPLE_NAME}.h)
	ENDIF()
	# imgui example requires additional source files
	IF(${EXAMPLE_NAME} STREQUAL "imgui")
		file(GLOB ADD_SOURCE "../external/imgui/*.cpp")
		SET(SOURCE ${SOURCE} ${ADD_SOURCE})
	ENDIF()
	# wayland requires additional source files
	IF(USE_WAYLAND_WSI)
		SET(SOURCE ${SOURCE} ${CMAKE_BINARY_DIR}/xdg-shell-client-protocol.h ${CMAKE_BINARY_DIR}/xdg-shell-protocol.c)
	ENDIF()
	# Add shaders
	set(SHADER_DIR_GLSL "../shaders/glsl/${EXAMPLE_NAME}")
	file(GLOB SHADERS_GLSL "${SHADER_DIR_GLSL}/*.vert" "${SHADER_DIR_GLSL}/*.frag" "${SHADER_DIR_GLSL}/*.comp" "${SHADER_DIR_GLSL}/*.geom" "${SHADER_DIR_GLSL}/*.tesc" "${SHADER_DIR_GLSL}/*.tese" "${SHADER_DIR_GLSL}/*.mesh" "${SHADER_DIR_GLSL}/*.task" "${SHADER_DIR_GLSL}/*.rgen" "${SHADER_DIR_GLSL}/*.rchit" "${SHADER_DIR_GLSL}/*.rmiss" "${SHADER_DIR_GLSL}/*.rcall" "${SHADER_DIR_GLSL}/*.rahit" "${SHADER_DIR_GLSL}/*.rint" "${SHADER_DIR_GLSL}/*.glsl")
	set(SHADER_DIR_HLSL "../shaders/hlsl/${EXAMPLE_NAME}")
	file(GLOB SHADERS_HLSL "${SHADER_DIR_HLSL}/*.vert" "${SHADER_DIR_HLSL}/*.frag" "${SHADER_DIR_HLSL}/*.comp" "${SHADER_DIR_HLSL}/*.geom" "${SHADER_DIR_HLSL}/*.tesc" "${SHADER_DIR_HLSL}/*.tese" "${SHADER_DIR_HLSL}/*.mesh" "${SHADER_DIR_HLSL}/*.task" "${SHADER_DIR_HLSL}/*.rgen" "${SHADER_DIR_HLSL}/*.rchit" "${SHADER_DIR_HLSL}/*.rmiss" "${SHADER_DIR_HLSL}/*.rcall" "${SHADER_DIR_HLSL}/*.rahit" "${SHADER_DIR_HLSL}/*.rint")
	source_group("Shaders\\GLSL" FILES ${SHADERS_GLSL})
	source_group("Shaders\\HLSL" FILES ${SHADERS_HLSL})
	# Add optional readme / tutorial
	file(GLOB README_FILES "${EXAMPLE_FOLDER}/*.md")
	if(WIN32)
		add_executable(${EXAMPLE_NAME} WIN32 ${MAIN_CPP} ${SOURCE} ${MAIN_HEADER} ${SHADERS_GLSL} ${SHADERS_HLSL} ${README_FILES})
		target_link_libraries(${EXAMPLE_NAME} base ${Vulkan_LIBRARY} ${WINLIBS})
	else(WIN32)
		add_executable(${EXAMPLE_NAME} ${MAIN_CPP} ${SOURCE} ${MAIN_HEADER} ${SHADERS_GLSL} ${SHADERS_HLSL} ${README_FILES})
		target_link_libraries(${EXAMPLE_NAME} base )
	endif(WIN32)
	# OCC lib
	file(GLOB LIB_FILES "../external/occt_vc14-64-combined/occt_vc14-64/win64/vc14/lib/*.lib")
	foreach(LIB ${LIB_FILES})
		target_link_libraries(${EXAMPLE_NAME} ${LIB})
	endforeach()

	# OCC dll
	file(GLOB DLL_FILES "../external/occt_vc14-64-combined/occt_vc14-64/win64/vc14/bin/*.dll")
	foreach(DLL ${DLL_FILES})
		add_custom_command(TARGET ${EXAMPLE_NAME} POST_BUILD COMMAND ${CMAKE_COMMAND} -E copy_if_different "${DLL}" "$<TARGET_FILE_DIR:${EXAMPLE_NAME}>")
	endforeach()

	# OCC 3rd party dll
	file(GLOB DLL_FILES "../external/occt_vc14-64-combined/3rdparty-vc14-64/ffmpeg-3.3.4-64/bin/*.dll")
	foreach(DLL ${DLL_FILES})
		add_custom_command(TARGET ${EXAMPLE_NAME} POST_BUILD COMMAND ${CMAKE_COMMAND} -E copy_if_different "${DLL}" "$<TARGET_FILE_DIR:${EXAMPLE_NAME}>")
	endforeach()
		
	add_custom_command(TARGET ${EXAMPLE_NAME} POST_BUILD COMMAND ${CMAKE_COMMAND} -E copy_if_different "${CMAKE_CURRENT_SOURCE_DIR}/../external/occt_vc14-64-combined/3rdparty-vc14-64/jemalloc-vc14-64/bin/jemalloc.dll" "$<TARGET_FILE_DIR:${EXAMPLE_NAME}>")
	add_custom_command(TARGET ${EXAMPLE_NAME} POST_BUILD COMMAND ${CMAKE_COMMAND} -E copy_if_different "${CMAKE_CURRENT_SOURCE_DIR}/../external/occt_vc14-64-combined/3rdparty-vc14-64/tbb-2021.13.0-x64/bin/tbb12.dll" "$<TARGET_FILE_DIR:${EXAMPLE_NAME}>")
	add_custom_command(TARGET ${EXAMPLE_NAME} POST_BUILD COMMAND ${CMAKE_COMMAND} -E copy_if_different "${CMAKE_CURRENT_SOURCE_DIR}/../external/occt_vc14-64-combined/3rdparty-vc14-64/freeimage-3.18.0-x64/bin/FreeImage.dll" "$<TARGET_FILE_DIR:${EXAMPLE_NAME}>")
	add_custom_command(TARGET ${EXAMPLE_NAME} POST_BUILD COMMAND ${CMAKE_COMMAND} -E copy_if_different "${CMAKE_CURRENT_SOURCE_DIR}/../external/occt_vc14-64-combined/3rdparty-vc14-64/freetype-2.13.3-x64/bin/freetype.dll" "$<TARGET_FILE_DIR:${EXAMPLE_NAME}>")
	add_custom_command(TARGET ${EXAMPLE_NAME} POST_BUILD COMMAND ${CMAKE_COMMAND} -E copy_if_different "${CMAKE_CURRENT_SOURCE_DIR}/../external/occt_vc14-64-combined/3rdparty-vc14-64/openvr-1.14.15-64/bin/win64/openvr_api.dll" "$<TARGET_FILE_DIR:${EXAMPLE_NAME}>")


	file(MAKE_DIRECTORY ${CMAKE_SOURCE_DIR}/bin)
	set_target_properties(${EXAMPLE_NAME} PROPERTIES VS_DEBUGGER_WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}/bin)
	if(${EXAMPLE_NAME} STREQUAL "texture3d")
		if(APPLE)
			# SRS - Use MacPorts paths as default since the same on x86 and Apple Silicon, can override for homebrew on cmake command line
			if(NOT OpenMP_omp_LIBRARY AND EXISTS /opt/local/lib/libomp/libomp.dylib)
				set(OpenMP_omp_LIBRARY /opt/local/lib/libomp/libomp.dylib)
			endif()
			if(CMAKE_C_COMPILER_ID MATCHES "Clang\$")
				set(OpenMP_C_FLAGS "-Xclang -fopenmp")
				set(OpenMP_C_LIB_NAMES "omp")
				if(NOT OpenMP_C_INCLUDE_DIR AND EXISTS /opt/local/include/libomp)
					set(OpenMP_C_INCLUDE_DIR /opt/local/include/libomp)
				endif()
			endif()
			if(CMAKE_CXX_COMPILER_ID MATCHES "Clang\$")
				set(OpenMP_CXX_FLAGS "-Xclang -fopenmp")
				set(OpenMP_CXX_LIB_NAMES "omp")
				if(NOT OpenMP_CXX_INCLUDE_DIR AND EXISTS /opt/local/include/libomp)
					set(OpenMP_CXX_INCLUDE_DIR /opt/local/include/libomp)
				endif()
			endif()
		endif()
		find_package(OpenMP)
		if(OpenMP_CXX_FOUND)
			set_target_properties(${EXAMPLE_NAME} PROPERTIES COMPILE_FLAGS ${OpenMP_CXX_FLAGS})
			target_include_directories(${EXAMPLE_NAME} PRIVATE ${OpenMP_CXX_INCLUDE_DIRS})
			target_link_libraries(${EXAMPLE_NAME} ${OpenMP_CXX_LIBRARIES})
		endif()
	endif()

	if(RESOURCE_INSTALL_DIR)
		install(TARGETS ${EXAMPLE_NAME} DESTINATION ${CMAKE_INSTALL_BINDIR})
	endif()
endfunction(buildExample)

# Build all examples
function(buildExamples)
	foreach(EXAMPLE ${EXAMPLES})
		buildExample(${EXAMPLE})
	endforeach(EXAMPLE)
endfunction(buildExamples)

set(EXAMPLES
	appelhiddenline
	bloom
	bufferdeviceaddress
	computecloth
	computecullandlod
	computeheadless
	computenbody
	computeparticles
	computeraytracing
	computeshader
	conditionalrender
	conservativeraster
	debugprintf
	debugutils
	deferred
	deferredmultisampling
	deferredshadows
	descriptorbuffer
	descriptorindexing
	descriptorsets
	displacement
	distancefieldfonts
	dynamicrendering
	dynamicstate
	dynamicuniformbuffer
	gbufferhiddenline	
	gears
	geometryshader
	gltfloading
	gltfscenerendering
	gltfskinning
	graphicspipelinelibrary
	hdr
	hostimagecopy
	imgui
	indirectdraw
	inlineuniformblocks
	inputattachments
	instancing
	meshshader
	multisampling
	multithreading
	multiview
	negativeviewportheight	
	occ
	occlusionquery
	occsilhouette
	offscreen
	oit
	parallaxmapping
	particlesystem
	pbrbasic
	pbribl
	pbrtexture
	pipelines
	pipelinestatistics
	pushconstants
	pushdescriptors
	radialblur
	rayquery
	raytracingbasic
	raytracingcallable
	raytracinggltf
	raytracingintersection
	raytracingreflections
	raytracingpositionfetch
	raytracingsbtdata
	raytracingshadows
	raytracingtextures
	renderheadless
	robertshiddenline
	screenshot
	shaderobjects
	shadowmapping
	shadowmappingomni
	shadowmappingcascade
	silhouettetest
	specializationconstants
	sphericalenvmapping
	ssao
	stencilbuffer
	subpasses
	terraintessellation
	tessellation
	textoverlay
	texture
	texture3d
	texturearray
	texturecubemap
	texturecubemaparray
	texturemipmapgen
	texturesparseresidency
	timelinesemaphore
	triangle
	trianglevulkan13
	variablerateshading
	vertexattributes
	viewportarray
	vulkanscene
)

buildExamples()
