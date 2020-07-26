# Warning! This code was automatically generated for projects
# without default 'CMakeLists.txt' file.

set(app_sources
	"main.c" "ble_mesh.c" "tasks.c" "example/blink_task.c" "wifi.c"
)

idf_component_register(SRCS ${app_sources})
