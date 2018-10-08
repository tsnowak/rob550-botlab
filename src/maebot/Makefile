include ../common.mk

CFLAGS  =  $(CFLAGS_STD)  $(CFLAGS_COMMON)  $(CFLAGS_LCMTYPES) -O3
CXXFLAGS = $(CXXFLAGS_STD) $(CFLAGS_COMMON)  $(CFLAGS_LCMTYPES) -O3
LDFLAGS = $(LDFLAGS_STD) $(LDFLAGS_COMMON) $(LDFLAGS_LCMTYPES)
LIBDEPS = $(call libdeps, $(LDFLAGS))

BIN_DRIVE_SQUARE = $(BIN_PATH)/drive_square
BIN_MAEBOT_DRIVER = $(BIN_PATH)/maebot_driver
BIN_MOTION_CONTROLLER = $(BIN_PATH)/motion_controller
BIN_ODOMETRY = $(BIN_PATH)/odometry
BIN_RPLIDAR_DRIVER = $(BIN_PATH)/rplidar_driver

ALL = $(BIN_MAEBOT_DRIVER) $(BIN_MOTION_CONTROLLER) $(BIN_RPLIDAR_DRIVER) $(BIN_ODOMETRY) $(BIN_DRIVE_SQUARE)

all: $(ALL)

$(BIN_DRIVE_SQUARE): drive_square.o $(LIBDEPS)
	@echo "    $@"
	@$(CXX) -o $@ $^ $(LDFLAGS)

$(BIN_MAEBOT_DRIVER): maebot_driver.o $(LIBDEPS)
	@echo "    $@"
	@$(CC) -o $@ $^ $(LDFLAGS)

$(BIN_MOTION_CONTROLLER): motion_controller.o $(LIBDEPS)
	@echo "    $@"
	@$(CXX) -o $@ $^ $(LDFLAGS)
	
$(BIN_ODOMETRY): odometry.o $(LIBDEPS)
	@echo "    $@"
	@$(CXX) -o $@ $^ $(LDFLAGS)

$(BIN_RPLIDAR_DRIVER): rplidar_driver.o rplidar.o $(LIBDEPS)
	@echo "    $@"
	@$(CC) -o $@ $^ $(LDFLAGS)
	
clean:
	@rm -f *.o *~ *.a
	@rm -f $(ALL)
