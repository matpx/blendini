
RAYLIB_PATH = raylib/src/
OBJS = blendini.o rjm.o imgui/imgui.o imgui/imgui_demo.o imgui/imgui_draw.o imgui/imgui_tables.o imgui/imgui_widgets.o rlImGui/rlImGui.o

blendini: $(OBJS) $(RAYLIB_PATH)libraylib.a $(IMGUI_SRC)
	$(CXX) -o blendini $^ 

FLAGS = -std=c++20 -Wall -Wextra -I$(RAYLIB_PATH) -Irjm/ -Iimgui/ -IrlImGui/

%.o: %.cpp
	$(CXX) -o $@ -c $^ $(FLAGS)

$(RAYLIB_PATH)libraylib.a:
	$(MAKE) -C $(RAYLIB_PATH)

.PHONY: clean

clean:
	$(RM) blendini blendini.exe
	$(RM) *.o imgui/*.o rlImGui/*.o
	$(MAKE) -C $(RAYLIB_PATH) clean

