if (NOT KORTEX_DIR)
	include(FetchContent)
	if (UNIX)
		FetchContent_Declare(
			kortex_api_rel
			URL  https://artifactory.kinovaapps.com:443/artifactory/generic-public/kortex/API/2.7.0/linux_x86-64_gcc_5.4.zip
			URL_HASH SHA256=e522fde1424103050c3512f7989c2e7be74353fc754a17d162bb33ad365fb7ae
		)
	elseif(WIN32)
		if(MSVC_TOOLSET_VERSION EQUAL "140") # VS 2015
			FetchContent_Declare(
				kortex_api_rel
				URL  https://artifactory.kinovaapps.com:443/artifactory/generic-public/kortex/API/2.7.0/windows_x86-64_msvc-2015.zip
				URL_HASH SHA256=b3733fc6c8271f73bfe5a1d5d7c7b4cf71f19fc1c54500ec571eb5052c603b04
				)
		elseif(MSVC_TOOLSET_VERSION EQUAL "141") # VS 2017
			FetchContent_Declare(
				kortex_api_rel
				URL  https://artifactory.kinovaapps.com:443/artifactory/generic-public/kortex/API/2.7.0/windows_x86-64_msvc-2017.zip
				URL_HASH SHA256=1b9c3cfca1a7f2709adaa368af495c1b780dfc849881ccb511ef4604e613949e
				)
		elseif(MSVC_TOOLSET_VERSION EQUAL "142") # VS 2019
			FetchContent_Declare(
				kortex_api_rel
				URL  https://artifactory.kinovaapps.com:443/artifactory/generic-public/kortex/API/2.7.0/windows_x86-64_msvc-2019.zip
				URL_HASH SHA256=92d9b2918d23f92c47a7011b69492a6742e69ba79f43e4221212b03174a67b52
				)
		else()
			message(FATAL_ERROR "unknown MSVC platform")
		endif()
	else()
		message(FATAL_ERROR "unknown platform")
	endif()

	FetchContent_MakeAvailable(kortex_api_rel)
	set(KORTEX_DIR ${kortex_api_rel_SOURCE_DIR} CACHE PATH "The path to the kinova library")
endif()
set(KORTEX_LIB_DIR "release")

add_library(kortex_api INTERFACE)
if(UNIX)
	target_link_libraries(kortex_api INTERFACE ${KORTEX_DIR}/lib/${KORTEX_LIB_DIR}/libKortexApiCpp.a)
elseif(WIN32)
	target_link_libraries(kortex_api INTERFACE ${KORTEX_DIR}/lib/${KORTEX_LIB_DIR}/KortexApiCpp.lib)
endif()
target_include_directories(kortex_api SYSTEM INTERFACE ${KORTEX_DIR}/include ${KORTEX_DIR}/include/client ${KORTEX_DIR}/include/common ${KORTEX_DIR}/include/messages ${KORTEX_DIR}/include/client_stubs)
if(UNIX)
	target_compile_definitions(kortex_api INTERFACE _OS_UNIX)
elseif(WIN32)
	target_compile_definitions(kortex_api INTERFACE _OS_WINDOWS NOMINMAX)
  if(MSVC)
		target_compile_options(kortex_api INTERFACE /bigobj)
  endif()
endif()


