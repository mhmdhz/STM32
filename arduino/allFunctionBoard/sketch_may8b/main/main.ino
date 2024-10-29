/*CheetahSystem

├── CheetahSystem.h
├── CheetahSystem.cpp
├── MQTTHandler.h
├── MQTTHandler.cpp
├── OBDHandler.h
├── OBDHandler.cpp
├── PowerManager.h
├── PowerManager.cpp
└── main.ino
*/
// main.ino

#include "CheetahSystem.h"

CheetahSystem cheetah;

void setup() {
  cheetah.initialize();
}

void loop() {
  cheetah.run();
}
