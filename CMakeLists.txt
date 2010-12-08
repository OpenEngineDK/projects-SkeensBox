# Project name. Should be the same as the project directory name and
# will become the name of the binary target.
SET( PROJECT_NAME "SkeensBox")

# Project source code list
SET( PROJECT_SOURCES
  main.cpp
)

# Project executable
ADD_EXECUTABLE(${PROJECT_NAME}
  ${PROJECT_SOURCES}
)

# Project dependencies
TARGET_LINK_LIBRARIES(${PROJECT_NAME}
  # Core library dependencies
  OpenEngine_Core
  OpenEngine_Logging
  OpenEngine_Display
  OpenEngine_Devices
  OpenEngine_Scene
  # Extension dependencies
  Extensions_SDL
  Extensions_SDLImage
  Extensions_SetupHelpers  
  Extensions_RBPhysics
  Extensions_MeshUtils
  Extensions_OpenGLSelection
  Extensions_GenericHandlers
  Extensions_AssimpResource
  Extensions_BagContainer
)

