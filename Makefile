
RAYLIB_SRC = raylib/src/

blendini: blendini.cpp $(RAYLIB_SRC)libraylib.a
	$(CXX) -o blendini blendini.cpp -std=c++20 -Wall -Wextra -I$(RAYLIB_SRC) $(RAYLIB_SRC)libraylib.a

$(RAYLIB_SRC)libraylib.a:
	$(MAKE) -C $(RAYLIB_SRC)

.PHONY: clean

clean:
	$(RM) blendini blendini.exe
	$(MAKE) -C $(RAYLIB_SRC) clean

