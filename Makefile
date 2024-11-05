SOURCE += src/
RAYLIB_PATH += $(SOURCE)extern/raylib/src/

OBJS += $(patsubst %.cpp,%.o,$(wildcard $(SOURCE)extern/imgui/*.cpp))
OBJS += $(patsubst %.cpp,%.o,$(wildcard $(SOURCE)extern/rlImGui/*.cpp))
OBJS += $(patsubst %.cpp,%.o,$(wildcard $(SOURCE)*.cpp))

FLAGS += -isystem $(SOURCE)extern/ -isystem $(SOURCE)extern/raylib/src/ -isystem $(SOURCE)extern/imgui/
FLAGS += -isystem $(SOURCE)extern/eigen/ -isystem $(SOURCE)extern/entt/src/ -isystem $(SOURCE)extern/thread-pool/include/
FLAGS += -std=c++20 -Wall -Wextra -Wpedantic -I$(RAYLIB_PATH)
FLAGS += -march=native

ifdef DEBUG
	FLAGS += -fsanitize=address,undefined -fno-omit-frame-pointer
	FLAGS += -D_GLIBCXX_ASSERTIONS
	FLAGS += -Og -g

	RAYLIB_MODE ?= DEBUG
else
	FLAGS += -O3

	ifdef NOSAFETY
		FLAGS += -DNDEBUG
	endif

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
	$(RM) $(SOURCE)*.o $(SOURCE)*.d $(SOURCE)*.o.tmp
	$(RM) $(SOURCE)extern/imgui/*.o $(SOURCE)extern/imgui/*.d $(SOURCE)extern/imgui/*.o.tmp
	$(RM) $(SOURCE)extern/rlImGui/*.o $(SOURCE)extern/rlImGui/*.d $(SOURCE)extern/rlImGui/*.o.tmp
	$(MAKE) -C $(RAYLIB_PATH) clean
