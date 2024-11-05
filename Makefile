RAYLIB_PATH += src/extern/raylib/src/

OBJS += $(patsubst %.cpp,%.o,$(wildcard src/extern/imgui/*.cpp))
OBJS += $(patsubst %.cpp,%.o,$(wildcard src/extern/rlImGui/*.cpp))
OBJS += $(patsubst %.cpp,%.o,$(wildcard src/*.cpp))

FLAGS += -isystem src/extern/ -isystem src/extern/raylib/src -isystem src/extern/imgui -isystem src/extern/eigen -isystem src/extern/entt/src -isystem src/extern/thread-pool/include
FLAGS += -std=c++20 -Wall -Wextra -Wpedantic -I$(RAYLIB_PATH)
FLAGS += -march=native

ifdef DEBUG
	FLAGS += -fsanitize=address,undefined -fno-omit-frame-pointer
	FLAGS += -D_GLIBCXX_ASSERTIONS
	FLAGS += -Og -g

	RAYLIB_MODE ?= DEBUG
else
	FLAGS += -O3 -DNDEBUG

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
	$(RM) src/extern/imgui/*.o src/extern/imgui/*.d src/extern/imgui/*.o.tmp
	$(RM) src/extern/rlImGui/*.o src/extern/rlImGui/*.d src/extern/rlImGui/*.o.tmp
	$(MAKE) -C $(RAYLIB_PATH) clean
