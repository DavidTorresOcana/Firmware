
px4_add_module(
	MODULE modules__dbx_control
	MAIN dbx_control
	STACK 1200
	COMPILE_FLAGS
		-Os
        
	SRCS
		dbx_control_main.c
        	dbx_control_params.c
	DEPENDS
		platforms__common
	)
# vim: set noet ft=cmake fenc=utf-8 ff=unix : 
