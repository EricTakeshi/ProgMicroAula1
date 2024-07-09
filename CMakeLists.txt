cmake_minimum_required(VERSION 3.15)

project(ProgMicroAula1 LANGUAGES C CXX ASM)

find_bsp(ID com.sysprogs.project_importers.stm32.cubemx SOURCE_PROJECT ProgMicroAula1.ioc)

add_bsp_based_executable(NAME ProgMicroAula1
	GENERATE_BIN
	GENERATE_MAP)

