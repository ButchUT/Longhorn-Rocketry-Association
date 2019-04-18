CONTROL_PATH = ../../Control/src

CURVE_GEN_PATH = ../../Curve\ Generation/C/lib
INTEGRATOR_PATH = $(CURVE_GEN_PATH)/verlet_integrator/src
ACCELCALC_PATH = $(CURVE_GEN_PATH)/acceleration_calculator/src
DRAGCALC_PATH = $(CURVE_GEN_PATH)/constant_area_drag_calculator/src
DENSITYCALC_PATH = $(CURVE_GEN_PATH)/density_calculator/src
INTERPOLATE_PATH = $(CURVE_GEN_PATH)/interpolate/src

HARDWARE_PATH = ../hardware
BAROMETER_PATH = $(HARDWARE_PATH)/barometer
IMU_PATH = $(HARDWARE_PATH)/imu

VEHICLE_PATH = ../vehicle

SRC = $(INTEGRATOR_PATH)/*.cc $(DRAGCALC_PATH)/*.cc $(ACCELCALC_PATH)/*.cc $(DENSITYCALC_PATH)/*.cc $(INTERPOLATE_PATH)/*.cc $(CONTROL_PATH)/*.cc $(BAROMETER_PATH)/*.cc $(IMU_PATH)/*.cc $(VEHICLE_PATH)/*.cc
HEADER = $(INTEGRATOR_PATH)/*.h $(DRAGCALC_PATH)/*.h $(ACCELCALC_PATH)/*.h $(DENSITYCALC_PATH)/*.h $(INTERPOLATE_PATH)/*.h $(CONTROL_PATH)/*.h $(BAROMETER_PATH)/*.h $(IMU_PATH)/*.h $(VEHICLE_PATH)/*.h

sac_sim: $(HEADER) $(SRC)
	g++ -std=c++11 -g $(SRC) stats.cc stats.h sac_sim.cc flight_sim.cc flight_sim.h noise.cc noise.h telemetry.cc telemetry.h -I./ -I$(INTEGRATOR_PATH) -I$(ACCELCALC_PATH) -I$(DRAGCALC_PATH) -I$(DENSITYCALC_PATH) -I$(INTERPOLATE_PATH) -I$(CONTROL_PATH) -I$(BAROMETER_PATH) -I$(IMU_PATH) -I$(VEHICLE_PATH) -o sac_sim

big_tester: $(HEADER) $(SRC)
	g++ -std=c++11 -g $(SRC) stats.cc stats.h big_tester.cc flight_sim.cc flight_sim.h noise.cc noise.h telemetry.cc telemetry.h -I./ -I$(INTEGRATOR_PATH) -I$(ACCELCALC_PATH) -I$(DRAGCALC_PATH) -I$(DENSITYCALC_PATH) -I$(INTERPOLATE_PATH) -I$(CONTROL_PATH) -I$(BAROMETER_PATH) -I$(IMU_PATH) -I$(VEHICLE_PATH) -o big_tester

clean:
	rm sac_sim big_tester *~
