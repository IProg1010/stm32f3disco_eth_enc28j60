cmake_minimum_required(VERSION 3.15.3)

# Optional: print out extra messages to see what is going on. Comment it to have less verbose messages
set(CMAKE_VERBOSE_MAKEFILE ON)

# Path to toolchain file. This one has to be before 'project()' below
set(CMAKE_TOOLCHAIN_FILE ${CMAKE_SOURCE_DIR}/toolchain.cmake)
set(GIT_HEAD_FILE "${CMAKE_CURRENT_SOURCE_DIR}/.git/HEAD")

project(eth_enj C ASM)

set(PATH_TO_SDK ${CMAKE_SOURCE_DIR}/../../cmsis CACHE PATH "Path to foled with sdk for chip of stm32f1X")

set(EXECUTABLE ${PROJECT_NAME}.elf)
set(LINKER_FILE ${CMAKE_SOURCE_DIR}/flash.ld)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_C_STANDARD 11)
set(CMAKE_ASM_FLAGS "-x assembler-with-cpp")
#set_property(SOURCE startup_stm32f103xb.s APPEND PROPERTY COMPILE_OPTIONS "-x" "assembler-with-cpp")

message("PAth to ${PATH_TO_SDK}")
message("Cmake source dir" ${CMAKE_SOURCE_DIR})

# List of source files
set(SRC_FILES 
	${CMAKE_SOURCE_DIR}/main.c
	${CMAKE_SOURCE_DIR}/system_stm32f3xx.c
	${CMAKE_SOURCE_DIR}/startup_stm32f303xc.s
	#${CMAKE_SOURCE_DIR}/periph/dma.c
	${CMAKE_SOURCE_DIR}/periph/spi.c
	${CMAKE_SOURCE_DIR}/periph/usart.c
	${CMAKE_SOURCE_DIR}/periph/timer.c
	${CMAKE_SOURCE_DIR}/driver/enc28j60.c
	#${CMAKE_SOURCE_DIR}/APP/lwip_user.c
	${CMAKE_SOURCE_DIR}/lwip_interface/lwip.c
	${CMAKE_SOURCE_DIR}/lwip_port/tcp_server_raw/tcp_server.c
	${CMAKE_SOURCE_DIR}/lwip_port/port/ethernetif.c
	${CMAKE_SOURCE_DIR}/lwip/src/netif/etharp.c
	#${CMAKE_SOURCE_DIR}/lwip/src/netif/slipif.c
	${CMAKE_SOURCE_DIR}/lwip/src/core/def.c
	#${CMAKE_SOURCE_DIR}/lwip/src/core/dhcp.c
	${CMAKE_SOURCE_DIR}/lwip/src/core/dns.c
	${CMAKE_SOURCE_DIR}/lwip/src/core/init.c
	${CMAKE_SOURCE_DIR}/lwip/src/core/mem.c
	${CMAKE_SOURCE_DIR}/lwip/src/core/memp.c
	${CMAKE_SOURCE_DIR}/lwip/src/core/netif.c
	${CMAKE_SOURCE_DIR}/lwip/src/core/pbuf.c
	${CMAKE_SOURCE_DIR}/lwip/src/core/raw.c
	${CMAKE_SOURCE_DIR}/lwip/src/core/stats.c
	${CMAKE_SOURCE_DIR}/lwip/src/core/sys.c
	${CMAKE_SOURCE_DIR}/lwip/src/core/tcp_in.c
	${CMAKE_SOURCE_DIR}/lwip/src/core/tcp_out.c
	${CMAKE_SOURCE_DIR}/lwip/src/core/tcp.c
	${CMAKE_SOURCE_DIR}/lwip/src/core/timers.c
	${CMAKE_SOURCE_DIR}/lwip/src/core/udp.c
	${CMAKE_SOURCE_DIR}/lwip/src/core/ipv4/autoip.c
	${CMAKE_SOURCE_DIR}/lwip/src/core/ipv4/icmp.c
	${CMAKE_SOURCE_DIR}/lwip/src/core/ipv4/igmp.c
	${CMAKE_SOURCE_DIR}/lwip/src/core/ipv4/inet_chksum.c
	${CMAKE_SOURCE_DIR}/lwip/src/core/ipv4/inet.c
	${CMAKE_SOURCE_DIR}/lwip/src/core/ipv4/ip_addr.c
	${CMAKE_SOURCE_DIR}/lwip/src/core/ipv4/ip_frag.c
	${CMAKE_SOURCE_DIR}/lwip/src/core/ipv4/ip.c
	#${CMAKE_SOURCE_DIR}/lwip_port/port/sys_arch.c
	)

# Build the executable based on the source files
add_executable(${EXECUTABLE} ${SRC_FILES})

# List of compiler defines, prefix with -D compiler option
target_compile_definitions(${EXECUTABLE} PRIVATE
	-D__DEBUG_MODE__
	)

# List of includ directories
target_include_directories(${EXECUTABLE} PUBLIC
	
	${CMAKE_SOURCE_DIR}/
	${CMAKE_SOURCE_DIR}/periph
	${CMAKE_SOURCE_DIR}/driver
	${CMAKE_SOURCE_DIR}/APP
	#${CMAKE_SOURCE_DIR}/net_core
	${CMAKE_SOURCE_DIR}/lwip_interface/
	${CMAKE_SOURCE_DIR}/lwip_port/tcp_server_raw
	${CMAKE_SOURCE_DIR}/lwip_port/port
	${CMAKE_SOURCE_DIR}/lwip_port/arch
	${CMAKE_SOURCE_DIR}/lwip/src/include
	${CMAKE_SOURCE_DIR}/lwip/src/include/lwip
	${CMAKE_SOURCE_DIR}/lwip/src/include/lwip/ipv4
	${CMAKE_SOURCE_DIR}/lwip/src/include/lwip/ipv6
	)

include_directories(
	${CMAKE_SOURCE_DIR}/
	${CMAKE_SOURCE_DIR}/periph
	${CMAKE_SOURCE_DIR}/driver
	${CMAKE_SOURCE_DIR}/APP
	#${CMAKE_SOURCE_DIR}/net_core
	${CMAKE_SOURCE_DIR}/lwip_interface/
	${CMAKE_SOURCE_DIR}/lwip_port/tcp_server_raw
	${CMAKE_SOURCE_DIR}/lwip_port/port
	${CMAKE_SOURCE_DIR}/lwip_port/arch
	${CMAKE_SOURCE_DIR}/lwip/src/include
	${CMAKE_SOURCE_DIR}/lwip/src/include/lwip
	${CMAKE_SOURCE_DIR}/lwip/src/include/ipv4/lwip)
	#${CMAKE_SOURCE_DIR}/lwip/src/include/lwip/ipv6) 

# Compiler options
target_compile_options(${EXECUTABLE} PRIVATE
	-mcpu=cortex-m4
	-mthumb
	-fdata-sections
	-ffunction-sections
	-Wall
	-O3
	)

# Linker options
target_link_options(${EXECUTABLE} PRIVATE
	-T${LINKER_FILE}
	-mcpu=cortex-m4
	-mthumb
	-specs=nano.specs
	-Wl,--gc-sections
	)

add_custom_command(TARGET ${EXECUTABLE}
	POST_BUILD
	COMMAND ${CMAKE_SIZE_UTIL} ${EXECUTABLE})

	execute_process(COMMAND git log --pretty=format:'%h' -n 1
	OUTPUT_VARIABLE GIT_REV
	ERROR_QUIET)

if ("${GIT_REV}" STREQUAL "")
	set(GIT_REV "N/A")
	set(GIT_DIFF "")
	set(GIT_TAG "N/A")
	set(GIT_BRANCH "N/A")
else()
	execute_process(
			COMMAND bash -c "git diff --quiet --exit-code || echo +"
			OUTPUT_VARIABLE GIT_DIFF)
	execute_process(
			COMMAND git describe --exact-match --tags
			OUTPUT_VARIABLE GIT_TAG ERROR_QUIET)
	execute_process(
		COMMAND git rev-parse --abbrev-ref HEAD
		OUTPUT_VARIABLE GIT_BRANCH)

	string(STRIP "${GIT_REV}" GIT_REV)
	string(SUBSTRING "${GIT_REV}" 1 7 GIT_REV)
	string(STRIP "${GIT_DIFF}" GIT_DIFF)
	string(STRIP "${GIT_TAG}" GIT_TAG)
	string(STRIP "${GIT_BRANCH}" GIT_BRANCH)
endif()

# Optional: Create hex, bin and S-Record files after the build
add_custom_command(TARGET ${EXECUTABLE}
        POST_BUILD
        COMMAND ${CMAKE_OBJCOPY} -O srec --srec-len=64 ${EXECUTABLE} ${PROJECT_NAME}.s19
        COMMAND ${CMAKE_OBJCOPY} -O ihex ${EXECUTABLE} ${PROJECT_NAME}.hex
        COMMAND ${CMAKE_OBJCOPY} -O binary ${EXECUTABLE} ${PROJECT_NAME}_${GIT_REV}.bin)
