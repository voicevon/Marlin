// M183 S0 Idle
// M183 S1 Pick_Up
// M183 S2 Place_Down
#include "../gcode.h"
#include "../../module/planner.h"

#define PIN_BRIDGE_A 35
#define PIN_BRIDGE_B 37

void GcodeSuite::M183() {
    pinMode(PIN_BRIDGE_A,OUTPUT);
    pinMode(PIN_BRIDGE_B,OUTPUT);
    planner.synchronize();
    if (parser.seen('S')) {
        const int a = parser.value_int();
        if (a == 1){
            digitalWrite(PIN_BRIDGE_A,LOW);
            digitalWrite(PIN_BRIDGE_B,HIGH);
            //SERIAL_ECHO_MSG("M183 S1 pickup");
        }
        else if (a == 2){
            digitalWrite(PIN_BRIDGE_A,HIGH);
            digitalWrite(PIN_BRIDGE_B,LOW);
            //SERIAL_ECHO_MSG("M183 S2 place down");
        }
        else{
            digitalWrite(PIN_BRIDGE_A,HIGH);
            digitalWrite(PIN_BRIDGE_B,HIGH);
        }
    }

}