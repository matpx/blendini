
RAYLIB_PATH = extern/raylib/src/
OBJS = blendini.o rjm.o extern/imgui/imgui.o extern/imgui/imgui_demo.o extern/imgui/imgui_draw.o extern/imgui/imgui_tables.o extern/imgui/imgui_widgets.o extern/rlImGui/rlImGui.o

FLAGS = -std=c++20 -O2 -g -Wall -Wextra -Wpedantic -I$(RAYLIB_PATH) -isystem extern/ -isystem extern/raylib/src -isystem extern/imgui -isystem extern/eigen

blendini: $(OBJS) $(RAYLIB_PATH)libraylib.a $(IMGUI_SRC)
	$(CXX) -o blendini $^ $(FLAGS)

%.o: %.cpp
	$(CXX) -o $@ -c $^ $(FLAGS)

$(RAYLIB_PATH)libraylib.a:
	$(MAKE) -C $(RAYLIB_PATH)

.PHONY: clean

clean:
	$(RM) blendini blendini.exe
	$(RM) *.o extern/imgui/*.o extern/rlImGui/*.o
	$(MAKE) -C $(RAYLIB_PATH) clean

