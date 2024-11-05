RAYLIB_PATH += extern/raylib/src/

OBJS += $(patsubst %.cpp,%.o,$(wildcard extern/imgui/*.cpp)) $(patsubst %.cpp,%.o,$(wildcard extern/rlImGui/*.cpp))
OBJS += $(patsubst %.cpp,%.o,$(wildcard src/*.cpp))

FLAGS += -isystem extern/ -isystem extern/raylib/src -isystem extern/imgui -isystem extern/eigen -isystem extern/entt/src -isystem extern/thread-pool/include
FLAGS += -std=c++20 -Wall -Wextra -Wpedantic -I$(RAYLIB_PATH)
FLAGS += -march=native

ifdef DEBUG
	FLAGS += -fsanitize=address,undefined -fno-omit-frame-pointer
	FLAGS += -D_GLIBCXX_ASSERTIONS
	FLAGS += -Og -g

	RAYLIB_MODE ?= DEBUG
else
	FLAGS += -O3

	RAYLIB_MODE ?= RELEASE
endif

blendini: $(OBJS) $(RAYLIB_PATH)libraylib.a
	$(CXX) -o blendini $^ $(FLAGS)

%.o: %.cpp
	$(CXX) -o $@ -c $< $(FLAGS)
	$(CXX) -MM -MT $@ -MF $(@:.o=.d) $< $(FLAGS)

-include $(OBJS:.o=.d)

$(RAYLIB_PATH)libraylib.a:
	$(MAKE) -C $(RAYLIB_PATH) RAYLIB_BUILD_MODE=$(RAYLIB_MODE)

.PHONY: clean

clean:
	$(RM) blendini blendini.exe
	$(RM) src/*.o src/*.d src/*.o.tmp
	$(RM) extern/imgui/*.o extern/imgui/*.d extern/imgui/*.o.tmp
	$(RM) extern/rlImGui/*.o extern/rlImGui/*.d extern/rlImGui/*.o.tmp
	$(MAKE) -C $(RAYLIB_PATH) clean
