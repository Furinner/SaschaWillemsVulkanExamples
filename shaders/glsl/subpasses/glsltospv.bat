cd %1
del *.spv
%VULKAN_SDK%\Bin\glslc composition.frag -o composition.frag.spv
%VULKAN_SDK%\Bin\glslc gbuffer.frag -o gbuffer.frag.spv
%VULKAN_SDK%\Bin\glslc transparent.frag -o transparent.frag.spv
%VULKAN_SDK%\Bin\glslc composition.vert -o composition.vert.spv
%VULKAN_SDK%\Bin\glslc gbuffer.vert -o gbuffer.vert.spv
%VULKAN_SDK%\Bin\glslc transparent.vert -o transparent.vert.spv