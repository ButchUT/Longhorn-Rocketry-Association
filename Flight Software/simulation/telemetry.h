#ifndef FLIGHTSIM_TELEMETRY
#define FLIGHTSIM_TELEMETRY

#include <map>
#include <fstream>
#include <iostream>
#include <string>

using namespace std;

class TelemetryPipeline {
protected:
  map<string, ofstream*> pipes;

public:
  TelemetryPipeline();

  ~TelemetryPipeline();

  void addPipe(string name, ofstream *target);

  void send(string pipe_name, string data);

  void sendln(string pipe_name, string data);

  ofstream* getPipe(string name);

  void close();
};

#endif
