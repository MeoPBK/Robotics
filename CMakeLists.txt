cmake_minimum_required(VERSION 2.8.7)
PROJECT(tutorialPlan)

set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/Modules)
set(CMAKE_CXX_STANDARD 11)
FIND_PACKAGE(Eigen REQUIRED)
FIND_PACKAGE(Boost REQUIRED)
find_package(Threads REQUIRED)

FIND_PACKAGE(Qt5 COMPONENTS OpenGL Core Widgets PrintSupport REQUIRED)
set(QT_USE_QTOPENGL TRUE)

# Find includes in corresponding build directories
set(CMAKE_INCLUDE_CURRENT_DIR ON)

# Instruct CMake to run moc automatically when needed.
set(CMAKE_AUTOMOC ON)

if (POLICY CMP0072)
	set(OpenGL_GL_PREFERENCE LEGACY)
endif()

FIND_PACKAGE(SoQt REQUIRED)
find_package(RL COMPONENTS KIN REQUIRED)
find_package(RL COMPONENTS PLAN REQUIRED)
find_package(RL COMPONENTS SG REQUIRED)

if(CMAKE_SIZEOF_VOID_P EQUAL 4)
	add_definitions(-DEIGEN_DONT_ALIGN)
endif(CMAKE_SIZEOF_VOID_P EQUAL 4)

SET(
	HDRS
        qt_visualization/QtWindow.h
        qt_visualization/QtViewer.h
        qt_visualization/QtPlanningThread.h
	RrtConConBase.h
	TutorialPlanSystem.h
        YourPlanner.h
	YourSampler.h
)

SET(
	SRCS
        qt_visualization/QtWindow.cpp
        qt_visualization/QtViewer.cpp
        qt_visualization/QtPlanningThread.cpp
	RrtConConBase.cpp
	tutorialPlan.cpp
	TutorialPlanSystem.cpp
        YourPlanner.cpp
	YourSampler.cpp
)

QT_WRAP_CPP(
        MOC_SRCS
        qt_visualization/QtWindow.h
        qt_visualization/QtViewer.h
        qt_visualization/QtPlanningThread.h
)

add_executable(
	tutorialPlan
	${HDRS}
	${SRCS}
    ${MOC_SRCS}
)


qt5_use_modules(tutorialPlan Widgets)

#target_compile_definitions(
add_definitions(
	"" ${QT_DEFINITIONS}
	-DSOQT_DLL
	-frounding-math
)

#target_include_directories(
include_directories(
	${QT_INCLUDES}
		${QT_QTOPENGL_INCLUDE_DIR} ${OPENGL_INCLUDE_DIR}
	${EIGEN_INCLUDE_DIRS}
	${RL_DIR}/../../include
)


TARGET_LINK_LIBRARIES(
	tutorialPlan
	${RL_LIBRARIES}
		Qt5::Core
		Qt5::Widgets
		Qt5::PrintSupport
		Qt5::OpenGL
		${CMAKE_THREAD_LIBS_INIT}
		${OPENGL_LIBRARIES}
	${QT_QTCORE_LIBRARY}
	${QT_QTGUI_LIBRARY}
	${QT_QTOPENGL_LIBRARY}
	${SOQT_LIBRARY_RELEASE}
	${COIN_LIBRARY_RELEASE}
)

