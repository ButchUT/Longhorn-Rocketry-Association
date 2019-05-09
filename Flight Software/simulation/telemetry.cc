#include "telemetry.h"

#include <fstream>

TelemetryPipeline::TelemetryPipeline() {}

TelemetryPipeline::~TelemetryPipeline() {}

void TelemetryPipeline::add_pipe(string name, ofstream *target) {
  pipes[name] = target;
}

void TelemetryPipeline::open_pipe(string name, string path) {
  std::ofstream *pipe = new ofstream();
  pipe->open(path);
  pipes[name] = pipe;
}

void TelemetryPipeline::send(string pipe_name, string data) {
  ofstream &pipe = *get_pipe(pipe_name);
  pipe << data;
}

void TelemetryPipeline::sendln(string pipe_name, string data) {
  send(pipe_name, data + "\n");
}

ofstream* TelemetryPipeline::get_pipe(string name) {
  return pipes[name];
}

void TelemetryPipeline::close() {
  for (auto const &pipe : pipes) {
    pipe.second->flush();
    pipe.second->close();
    delete pipe.second;
  }
}
