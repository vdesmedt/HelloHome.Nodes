#ifndef messages
#define messages

#include <Arduino.h>

#define FEAT_ENV    1
#define FEAT_BATT   2
#define FEAT_PULSE1 4
#define FEAT_PULSE2 8
#define FEAT_PULSE3 16

typedef struct BaseMessage {
  uint8_t msgType;
} BaseMessage;

#define RPT 0

typedef struct NodeStartedReport {
  const uint8_t msgType = RPT + (1 << 2);
  const uint8_t nodeType = 1;
  uint8_t signature[8];
  char version[8];
  uint16_t startCount;
} NodeStartedReport;

typedef struct NodeInfoReport {
  const uint8_t msgType = RPT + (2 << 2);
  uint16_t sendErrorCount;
  uint16_t vIn;
  bool dirty = false;
} NodeInfoReport;

typedef struct EnvironmentReport {
  const uint8_t msgType = RPT + (3 << 2);
  int temperature;
  int humidity;
  int pressure;
  bool dirty = false;
} EnvironmentReport;

typedef struct PulseReport {
  const uint8_t msgType = RPT + (4 << 2);
  uint16_t newPulses1;
  uint16_t newPulses2;
  uint16_t newPulses3;  
  bool dirty = false;
} PulseReport;


#define CMD 2

typedef struct NodeConfigCommand {
	const uint8_t msgType = CMD + (0 << 2);
	uint8_t signature[8];
	uint16_t newNodeId;
  uint16_t features;
  uint8_t nodeInfoFreq;
  uint8_t environmentFreq;
} NodeConfigCommand;

typedef struct RestartCommand {
  const uint8_t msgType = CMD + (1 << 2);
} RestartCommand;

#endif
