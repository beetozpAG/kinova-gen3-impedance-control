set(BOOST_INCLUDE_LIBRARIES numeric/odeint asio iostreams filesystem)
set(BOOST_ENABLE_CMAKE ON)
include(FetchContent)
# Add boost lib sources
# Download and extract the boost library from GitHub
message(STATUS "Downloading and extracting boost library sources. This will take some time...")
include(FetchContent)
set(FETCHCONTENT_QUIET FALSE) # Needed to print downloading progress
FetchContent_Declare(
    Boost
    URL https://github.com/boostorg/boost/releases/download/boost-1.87.0/boost-1.87.0-cmake.7z # downloading a zip release speeds up the download
    USES_TERMINAL_DOWNLOAD TRUE 
    GIT_PROGRESS TRUE   
    DOWNLOAD_NO_EXTRACT FALSE
		SYSTEM
)
FetchContent_MakeAvailable(Boost)
