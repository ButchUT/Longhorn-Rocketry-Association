LIB_PATH = ../../
VERLET_PATH = $(LIB_PATH)/verlet_integrator/src
ACCELCALC_PATH = $(LIB_PATH)/acceleration_calculator/src
DRAGCALC_PATH = $(LIB_PATH)/constant_area_drag_calculator/src
DENSITYCALC_PATH = $(LIB_PATH)/density_calculator/src
INTERPOLATE_PATH = $(LIB_PATH)/interpolate/src

SRC = *.cc $(VERLET_PATH)/verlet_integrator.cc $(DRAGCALC_PATH)/constant_area_drag_calculator.cc $(DENSITYCALC_PATH)/density_calculator.cc $(INTERPOLATE_PATH)/interpolate.cc
SRC_H = *.h $(ACCELCALC_PATH)/acceleration_calculator.h $(VERLET_PATH)/verlet_integrator.h $(DRAGCALC_PATH)/constant_area_drag_calculator.h $(DENSITYCALC_PATH)/density_calculator.h $(INTERPOLATE_PATH)/interpolate.h $(SRC)

sim: $(SRC_H)
	g++ -std=c++11 -I$(VERLET_PATH) -I$(ACCELCALC_PATH) -I$(DRAGCALC_PATH) -I$(DENSITYCALC_PATH) -I$(INTERPOLATE_PATH) $(SRC) -o sim

clean:
	rm sim *~
