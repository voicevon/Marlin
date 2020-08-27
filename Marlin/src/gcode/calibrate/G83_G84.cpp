
// Install PrintRun onto Ubunbu
// pip install --user wxpython
// https://zoomadmin.com/HowToInstall/UbuntuPackage/printrun


#include "../gcode.h"
#include "../../module/motion.h"
#include "../../module/planner.h"

#include "../../MarlinCore.h"

#define JOINT5_ENDSTOP_PIN 53

//Home Joint5 as axis E0 in Marlin
void GcodeSuite::G83(){
    
    while (false){
        //next version
        //Wait queue to be empty
    
    }
    destination.x = current_position.x;
    destination.y = current_position.y;
    destination.z = current_position.z;
    // SERIAL_ERROR_MSG("G83 homing...\n");
    pinMode(JOINT5_ENDSTOP_PIN, INPUT_PULLUP);
    delay(200);
    bool triggered = false;
    bool last_triggered_1 = false;
    bool last_triggered_2 = false;
    while (!triggered || !last_triggered_1 || !last_triggered_2){
        prepare_line_to_destination();
        current_position.e = 0;
        destination.e = -0.1 ;       
        sync_plan_position_e();   
        last_triggered_2 = last_triggered_1;
        // planner.synchronize();
        last_triggered_1 = triggered;
        triggered = digitalRead(JOINT5_ENDSTOP_PIN);
    }
    SERIAL_ECHO_MSG("G83 home-Joint5 is triggered ...\n");

    prepare_line_to_destination();

    current_position.e = E1_MIN_POS;   // For joint5 PFT-1901
    destination.e = E1_MIN_POS;
    sync_plan_position_e();
}

void GcodeSuite::G84(){


}