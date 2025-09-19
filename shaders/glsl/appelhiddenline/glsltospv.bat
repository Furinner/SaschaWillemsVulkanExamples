cd %1
del *.spv
for %%f in (*.frag) do (
    %VULKAN_SDK%\Bin\glslc %%~nxf -o %%~nxf.spv
)
for %%f in (*.vert) do (
    %VULKAN_SDK%\Bin\glslc %%~nxf -o %%~nxf.spv
)
for %%f in (*.comp) do (
    %VULKAN_SDK%\Bin\glslc %%~nxf -o %%~nxf.spv
)