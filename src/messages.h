#ifndef messages
#define messages

#include <Arduino.h>

#define RPT 0
#define CMD 2

#define FEAT_SI7021 1
#define FEAT_BMP    2
#define FEAT_VIN    4
#define FEAT_HAL1   8
#define FEAT_HAL2   16
#define FEAT_DRY    32

typedef struct BaseMessage {
  uint8_t msgType;
} BaseMessage;

typedef struct NodeInfoReport {
  const uint8_t msgType = RPT + (0 << 2);
  uint16_t sendErrorCount;
  uint16_t startCount;
  uint16_t vIn;
} NodeInfoReport;

typedef struct PulseReport {
  const uint8_t msgType = RPT + (1 << 2);
  uint8_t subNode;
  uint16_t newPulses;
} PulseReport;

typedef struct EnvironmentReport {
  const uint8_t msgType = RPT + (2 << 2);
  int temperature;
  int humidity;
  int pressure;
} EnvironmentReport;

typedef struct NodeStartedReport {
  const uint8_t msgType = RPT + (3 << 2);
  const uint8_t nodeType = 1;
  uint8_t signature[8];
  char version[7];
} NodeStartedReport;

typedef struct ReadyForProgCommand {
  const uint8_t msgType = RPT + (4 << 2);
} ReadyForProgCommand;

typedef struct NodeConfigCommand {
	const uint8_t msgType = CMD + (0 << 2);
	uint8_t signature[8];
	uint8_t newNodeId;
  uint16_t features;
  uint8_t nodeInfoFreq;
  uint8_t environmentFreq;
} NodeConfigCommand;

#endif
