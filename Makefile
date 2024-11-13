SOURCE = src/
RAYLIB_PATH = $(SOURCE)extern/raylib/src/
ARCH = -march=native

OBJS  = $(patsubst %.cpp,%.o,$(wildcard $(SOURCE)extern/imgui/*.cpp))
OBJS += $(patsubst %.cpp,%.o,$(wildcard $(SOURCE)extern/rlImGui/*.cpp))
OBJS += $(patsubst %.cpp,%.o,$(wildcard $(SOURCE)*.cpp))

CFLAGS  = -isystem $(SOURCE)extern/ -isystem $(SOURCE)extern/raylib/src/ -isystem $(SOURCE)extern/raylib-cpp/include/ -isystem $(SOURCE)extern/imgui/
CFLAGS += -isystem $(SOURCE)extern/eigen/ -isystem $(SOURCE)extern/entt/src/ -isystem $(SOURCE)extern/thread-pool/include/
CFLAGS += -std=c++20 -Wall -Wextra -Wpedantic -I$(RAYLIB_PATH)
CFLAGS += $(ARCH)

SAFETY_FLAGS += -fsanitize=address,undefined -fno-omit-frame-pointer
SAFETY_FLAGS += -D_GLIBCXX_ASSERTIONS -U_FORTIFY_SOURCE -D_FORTIFY_SOURCE=3

ifdef DEBUG
	CFLAGS += -Og -g -D_DEBUG
	CFLAGS += $(SAFETY_FLAGS)

	RAYLIB_MODE = DEBUG
	RAYLIB_CUSTOM_CFLAGS = $(ARCH) $(SAFETY_FLAGS)
else
	CFLAGS += -O3 -DNDEBUG -flto

	RAYLIB_MODE = RELEASE
	RAYLIB_CUSTOM_CFLAGS = $(ARCH) -flto
endif

blendini: $(OBJS) $(RAYLIB_PATH)libraylib.a
	$(CXX) -o blendini $^ $(CFLAGS)

%.o: %.cpp
	$(CXX) -o $@ -c $< $(CFLAGS)
	$(CXX) -MM -MT $@ -MF $(@:.o=.d) $< $(CFLAGS)

-include $(OBJS:.o=.d)

$(RAYLIB_PATH)libraylib.a:
	$(MAKE) -C $(RAYLIB_PATH) RAYLIB_BUILD_MODE=$(RAYLIB_MODE) CUSTOM_CFLAGS="$(RAYLIB_CUSTOM_CFLAGS)"

.PHONY: clean

clean:
	$(RM) blendini blendini.exe
	$(RM) $(SOURCE)*.o $(SOURCE)*.d $(SOURCE)*.o.tmp
	$(RM) $(SOURCE)extern/imgui/*.o $(SOURCE)extern/imgui/*.d $(SOURCE)extern/imgui/*.o.tmp
	$(RM) $(SOURCE)extern/rlImGui/*.o $(SOURCE)extern/rlImGui/*.d $(SOURCE)extern/rlImGui/*.o.tmp
	$(MAKE) -C $(RAYLIB_PATH) clean
