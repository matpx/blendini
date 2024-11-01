
RAYLIB_PATH = extern/raylib/src/
OBJS = blendini.o rjm.o extern/imgui/imgui.o extern/imgui/imgui_demo.o extern/imgui/imgui_draw.o extern/imgui/imgui_tables.o extern/imgui/imgui_widgets.o extern/rlImGui/rlImGui.o

blendini: $(OBJS) $(RAYLIB_PATH)libraylib.a $(IMGUI_SRC)
	$(CXX) -o blendini $^ 

FLAGS = -std=c++20 -Wall -Wextra -I$(RAYLIB_PATH) -Iextern/ -Iextern/raylib/src -Iextern/imgui -Iextern/eigen

%.o: %.cpp
	$(CXX) -o $@ -c $^ $(FLAGS)

$(RAYLIB_PATH)libraylib.a:
	$(MAKE) -C $(RAYLIB_PATH)

.PHONY: clean

clean:
	$(RM) blendini blendini.exe
	$(RM) *.o extern/imgui/*.o extern/rlImGui/*.o
	$(MAKE) -C $(RAYLIB_PATH) clean

