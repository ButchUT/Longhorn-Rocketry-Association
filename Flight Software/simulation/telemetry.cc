#include "telemetry.h"

TelemetryPipeline::TelemetryPipeline() {}

TelemetryPipeline::~TelemetryPipeline() {}

void TelemetryPipeline::addPipe(string name, ofstream *target) {
  pipes[name] = target;
}

void TelemetryPipeline::send(string pipe_name, string data) {
  ofstream &pipe = *getPipe(pipe_name);
  pipe << data;
}

void TelemetryPipeline::sendln(string pipe_name, string data) {
  send(pipe_name, data + "\n");
}

ofstream* TelemetryPipeline::getPipe(string name) {
  return pipes[name];
}

void TelemetryPipeline::close() {
  for (auto const &pipe : pipes) {
    pipe.second->flush();
    pipe.second->close();
  }
}
