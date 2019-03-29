#ifndef FLIGHTSIM_TELEMETRY
#define FLIGHTSIM_TELEMETRY

#include <map>
#include <fstream>
#include <iostream>
#include <string>

using namespace std;

/**
  A mapping of strings to file streams. Used to write simulator telemetry to
  output files.
*/
class TelemetryPipeline {
protected:
  map<string, ofstream*> pipes;

public:
  TelemetryPipeline();

  ~TelemetryPipeline();

  /**
    @brief Adds a new output mapping
  */
  void add_pipe(string name, ofstream *target);

  /**
    @brief Adds a new output mapping by opening an ofstream
  */
  void open_pipe(string name, string path);

  /**
    @brief Sends a string to an output
  */
  void send(string pipe_name, string data);

  /**
    @brief Sends a string to an output with an attached newline
  */
  void sendln(string pipe_name, string data);

  /**
    @brief Fetch an output by name
  */
  ofstream* get_pipe(string name);

  /**
    @brief Flushes, closes and frees all output streams.
  */
  void close();
};

#endif
