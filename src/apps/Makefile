include ../common.mk

# flags for building the gtk library
CFLAGS = $(CFLAGS_STD) \
	 $(CFLAGS_VX_GTK) \
         $(CFLAGS_IMAGESOURCE) \
	 $(CFLAGS_MATH) \
	 $(CFLAGS_COMMON) \
	 $(CFLAGS_LCMTYPES) \
	 -O3

CXXFLAGS = $(CXXFLAGS_STD) \
	$(CFLAGS_VX_GTK) \
	$(CFLAGS_GTK) \
	$(CFLAGS_IMAGESOURCE) \
	$(CFLAGS_MATH) \
	$(CFLAGS_COMMON) \
	$(CFLAGS_LCMTYPES) \
	-O3
	 
LDFLAGS = $(LDFLAGS_STD) \
	  $(LDFLAGS_VX_GTK) \
	  $(LDFLAGS_GTK) \
	  $(LDFLAGS_IMAGESOURCE) \
	  $(LDFLAGS_MATH) \
	  $(LDFLAGS_LCMTYPES) \
	  $(LDFLAGS_LCM) \
	  $(LDFLAGS_MAPPING) \
	  $(LDFLAGS_COMMON) \
	  $(LDFLAGS_PLANNING) 

LIBDEPS = $(call libdeps, $(LDFLAGS))

CHALLENGEDIR = challenge
CHALLENGEOBJ = challenge_gui.o challenge_gui_main.o challenge_results.o grid_visualization.o map_statistics.o

BOTGUIDIR = botgui
BOTGUIOBJ = botgui.o botgui_main.o

UTILSDIR = utils
UTILSOBJ = drawing_functions.o vx_gtk_window_base.o vx_utils.o

VPATH = $(CHALLENGEDIR) $(BOTGUIDIR) $(UTILSDIR)

BIN_CHALLENGE_GUI = $(BIN_PATH)/challenge_gui
BIN_BOTGUI = $(BIN_PATH)/botgui

ALL = $(BIN_CHALLENGE_GUI) $(BIN_BOTGUI)

all: $(ALL)
	
$(BIN_BOTGUI): $(BOTGUIOBJ) $(UTILSOBJ) $(LIBDEPS)
	@echo "    $@"
	@$(CXX) -o $@ $^ $(LDFLAGS)

$(BIN_CHALLENGE_GUI): $(CHALLENGEOBJ) $(UTILSOBJ) $(LIBDEPS)
	@echo "    $@"
	@$(CXX) -o $@ $^ $(LDFLAGS)

clean:
	@rm -f *.o *~ *.a $(CHALLENGEDIR)/*.o $(BOTGUIDIR)/*.o $(UTILSDIR)/*.o
	@rm -f $(ALL)
