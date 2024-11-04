
RAYLIB_PATH += extern/raylib/src/

OBJS += $(patsubst %.cpp,%.o,$(wildcard extern/imgui/*.cpp)) $(patsubst %.cpp,%.o,$(wildcard extern/rlImGui/*.cpp))
OBJS += $(patsubst %.cpp,%.o,$(wildcard src/*.cpp))

HEADER += $(wildcard extern/rjm/*.h) $(wildcard src/*.hpp)

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

blendini: $(OBJS) $(RAYLIB_PATH)libraylib.a $(IMGUI_SRC)
	$(CXX) -o blendini $^ $(FLAGS)

%.o: %.cpp $(HEADER)
	$(CXX) -o $@ -c $< $(FLAGS)

$(RAYLIB_PATH)libraylib.a:
	$(MAKE) -C $(RAYLIB_PATH) RAYLIB_BUILD_MODE=$(RAYLIB_MODE)

.PHONY: clean

clean:
	$(RM) blendini blendini.exe
	$(RM) src/*.o extern/imgui/*.o extern/rlImGui/*.o
	$(MAKE) -C $(RAYLIB_PATH) clean

