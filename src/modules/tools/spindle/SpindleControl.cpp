/*
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.
*/

#include "libs/Module.h"
#include "libs/Kernel.h"
#include "Gcode.h"
#include "Conveyor.h"
#include "SpindleControl.h"
#include "PublicDataRequest.h"

void SpindleControl::on_gcode_received(void *argument) 
{
    
    Gcode *gcode = static_cast<Gcode *>(argument);
        
    if (gcode->has_m)
    {
        if (gcode->m == 957)
        {
            // M957: report spindle speed
            report_speed();
        }
        else if (gcode->m == 958)
        {
            THECONVEYOR->wait_for_idle();
            // M958: set spindle PID parameters
            if (gcode->has_letter('P'))
                set_p_term( gcode->get_value('P') );
            if (gcode->has_letter('I'))
                set_p_term( gcode->get_value('I') );
            if (gcode->has_letter('D'))
                set_p_term( gcode->get_value('D') );
            // report PID settings
            get_pid_settings();
          
        }
        else if (gcode->m == 3 || gcode->m == 4) //TODO now M3 (CW) and M4 (CCW) is also supported, but runs the same direction
        {
            THECONVEYOR->wait_for_idle();
            // M3: Spindle on
            if(!spindle_on) {
                turn_on();
            }
            
            // M3 with S value provided: set speed
            if (gcode->has_letter('S'))
            {
                set_speed(gcode->get_value('S'));
            } else set_speed(spindle_target_rpm);

            wait_for_spindle(); //waits for spindle to start
        }
        else if (gcode->m == 5)
        {
            THECONVEYOR->wait_for_idle();
            // M5: spindle off
            if(spindle_on) {
                turn_off();
            }
        }
    }

}

void SpindleControl::on_get_public_data(void *argument)
{
    PublicDataRequest *pdr = static_cast<PublicDataRequest *>(argument);

    if(!pdr->starts_with(spindel_control_data_checksum)) return;

    struct t_spindle_state *t= static_cast<t_spindle_state*>(pdr->get_data_ptr());
    t->onstate = spindle_on;
    t->target_speed = spindle_target_rpm;
    pdr->set_taken();

}

void SpindleControl::on_set_public_data(void *argument)
{
    PublicDataRequest *pdr = static_cast<PublicDataRequest *>(argument);

    if(!pdr->starts_with(spindel_control_data_checksum)) return;

    struct t_spindle_state *t = static_cast<t_spindle_state*>(pdr->get_data_ptr());

    THECONVEYOR->wait_for_idle();
    if(t->onstate) {
        if(!spindle_on) {
            turn_on();
        }
        set_speed(spindle_target_rpm);
        spindle_target_rpm = t->target_speed;
        wait_for_spindle(); //waits for spindle to start
    } else {
        if(spindle_on) {
            turn_off();
        }
    }

    pdr->set_taken();
}

