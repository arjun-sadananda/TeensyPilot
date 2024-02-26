OS = LINUX
#OS = MACOSX
#OS = MACOSX_CLANG
#OS = WINDOWS

ifeq ($(OS), LINUX)
ALL = MotionCal imuread
CC = gcc
CXX = g++
CFLAGS = -O2 -Wall -D$(OS)
WXCONFIG = ~/wxwidgets/3.0.2.gtk2-opengl/bin/wx-config
WXFLAGS = `$(WXCONFIG) --cppflags`
CXXFLAGS = $(CFLAGS) `$(WXCONFIG) --cppflags`
LDFLAGS =
SFLAG = -s
CLILIBS = -lglut -lGLU -lGL -lm
MAKEFLAGS = --jobs=12

else ifeq ($(OS), MACOSX)
ALL = MotionCal.dmg
CC = gcc-4.2
CXX = g++-4.2
CFLAGS = -O2 -Wall -D$(OS)
WXCONFIG = ~/wxwidgets/3.0.2.mac-opengl/bin/wx-config
WXFLAGS = `$(WXCONFIG) --cppflags`
CXXFLAGS = $(CFLAGS) `$(WXCONFIG) --cppflags`
SFLAG = -s
CLILIBS = -lglut -lGLU -lGL -lm
VERSION = 0.01

else ifeq ($(OS), MACOSX_CLANG)
ALL = MotionCal.app
CC = /usr/bin/clang
CXX = /usr/bin/clang++
CFLAGS = -O2 -Wall -DMACOSX
WXCONFIG = wx-config
WXFLAGS = `$(WXCONFIG) --cppflags`
CXXFLAGS = $(CFLAGS) `$(WXCONFIG) --cppflags`
SFLAG =
CLILIBS = -lglut -lGLU -lGL -lm
VERSION = 0.01

else ifeq ($(OS), WINDOWS)
ALL = MotionCal.exe
#MINGW_TOOLCHAIN = i586-mingw32msvc
MINGW_TOOLCHAIN = i686-w64-mingw32
CC = $(MINGW_TOOLCHAIN)-gcc
CXX = $(MINGW_TOOLCHAIN)-g++
WINDRES = $(MINGW_TOOLCHAIN)-windres
CFLAGS = -O2 -Wall -D$(OS)
WXFLAGS = `$(WXCONFIG) --cppflags`
CXXFLAGS = $(CFLAGS) $(WXFLAGS)
LDFLAGS = -static -static-libgcc
SFLAG = -s
#WXCONFIG = ~/wxwidgets/3.0.2.mingw-opengl-i586/bin/wx-config
#WXCONFIG = ~/wxwidgets/3.0.2.mingw-opengl/bin/wx-config
WXCONFIG = ~/wxwidgets/3.1.0.mingw-opengl/bin/wx-config
CLILIBS = -lglut32 -lglu32 -lopengl32 -lm
MAKEFLAGS = --jobs=12

endif

OBJS = visualize.o serialdata.o rawdata.o magcal.o matrix.o fusion.o quality.o mahony.o
IMGS = checkgreen.png checkempty.png checkemptygray.png

all: $(ALL)

MotionCal: gui.o portlist.o images.o $(OBJS)
	$(CXX) $(SFLAG) $(CFLAGS) $(LDFLAGS) -o $@ $^ `$(WXCONFIG) --libs all,opengl`

MotionCal.exe: resource.o gui.o portlist.o images.o $(OBJS)
	$(CXX) $(SFLAG) $(CFLAGS) $(LDFLAGS) -o $@ $^ `$(WXCONFIG) --libs all,opengl`
	-pjrcwinsigntool $@
	-./cp_windows.sh $@

resource.o: resource.rc icon.ico
	$(WINDRES) $(WXFLAGS) -o resource.o resource.rc

images.cpp: $(IMGS) png2c.pl
	perl png2c.pl $(IMGS) > images.cpp

MotionCal.app: MotionCal Info.plist icon.icns
	mkdir -p $@/Contents/MacOS
	mkdir -p $@/Contents/Resources/English.lproj
	sed "s/1.234/$(VERSION)/g" Info.plist > $@/Contents/Info.plist
	/bin/echo -n 'APPL????' > $@/Contents/PkgInfo
	cp $< $@/Contents/MacOS/
	cp icon.icns $@/Contents/Resources/
	-pjrcmacsigntool $@
	touch $@

MotionCal.dmg: MotionCal.app
	mkdir -p dmg_tmpdir
	cp -r $< dmg_tmpdir
	hdiutil create -ov -srcfolder dmg_tmpdir -megabytes 20 -format UDBZ -volname MotionCal $@

imuread: imuread.o $(OBJS)
	$(CC) -s $(CFLAGS) $(LDFLAGS) -o $@ $^ $(CLILIBS)

clean:
	rm -f gui MotionCal imuread *.o *.exe *.sign? images.cpp
	rm -rf MotionCal.app MotionCal.dmg .DS_Store dmg_tmpdir

gui.o: gui.cpp gui.h imuread.h Makefile
portlist.o: portlist.cpp gui.h Makefile
imuread.o: imuread.c imuread.h Makefile
visualize.o: visualize.c imuread.h Makefile
serialdata.o: serialdata.c imuread.h Makefile
rawdata.o: rawdata.c imuread.h Makefile
magcal.o: magcal.c imuread.h Makefile
matrix.o: matrix.c imuread.h Makefile
fusion.o: fusion.c imuread.h Makefile
quality.o: quality.c imuread.h Makefile
mahony.o: mahony.c imuread.h Makefile

