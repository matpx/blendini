
RAYLIB_PATH += extern/raylib/src/

OBJS += extern/imgui/imgui.o extern/imgui/imgui_demo.o extern/imgui/imgui_draw.o extern/imgui/imgui_tables.o extern/imgui/imgui_widgets.o extern/rlImGui/rlImGui.o
OBJS += blendini.o rjm.o scene.o

HEADER += extern/rjm/rjm_raytrace.h
HEADER += raymath_eigen.hpp scene.hpp

FLAGS += -std=c++20 -Wall -Wextra -Wpedantic -I$(RAYLIB_PATH) -isystem extern/ -isystem extern/raylib/src -isystem extern/imgui -isystem extern/eigen -isystem extern/entt/src
FLAGS += -march=native

ifdef DEBUG
	FLAGS += -fsanitize=address,undefined -fno-omit-frame-pointer
	FLAGS += -Og -g

	RAYLIB_MODE ?= DEBUG
else
	FLAGS += -O3

	RAYLIB_MODE ?= RELEASE
endif

blendini: $(OBJS) $(RAYLIB_PATH)libraylib.a $(IMGUI_SRC)
	$(CXX) -o blendini $^ $(FLAGS)

%.o: %.cpp $(HEADER)
	$(CXX) -o $@ -c $< $(FLAGS)

$(RAYLIB_PATH)libraylib.a:
	$(MAKE) -C $(RAYLIB_PATH) RAYLIB_BUILD_MODE=$(RAYLIB_MODE)

.PHONY: clean

clean:
	$(RM) blendini blendini.exe
	$(RM) *.o extern/imgui/*.o extern/rlImGui/*.o
	$(MAKE) -C $(RAYLIB_PATH) clean

