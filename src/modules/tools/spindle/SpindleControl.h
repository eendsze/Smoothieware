/*
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef SPINDLE_CONTROL_MODULE_H
#define SPINDLE_CONTROL_MODULE_H

#include "checksumm.h"

#include "libs/Module.h"

#define spindel_control_data_checksum    CHECKSUM("spindle_control_data")
#define spindle_checksum                    CHECKSUM("spindle")
#define startuptime_checksum                CHECKSUM("startup_time")

struct t_spindle_state {
     int target_speed;
     bool onstate;
};

class SpindleControl: public Module {

public:
        SpindleControl() {};
        virtual ~SpindleControl() {};
        virtual void on_module_loaded() {};

    protected:
        bool spindle_on;
        int spindle_target_rpm;
        uint32_t delay_ms = 2000;

    private:
        void on_gcode_received(void *argument);
        void on_get_public_data(void* argument);
        void on_set_public_data(void* argument);
        void on_halt(void* argument);
        
        virtual void turn_on(void) {};
        virtual void turn_off(void) {};
        virtual void set_speed(int) {};
        virtual void report_speed(void) {};
        virtual void set_p_term(float) {};
        virtual void set_i_term(float) {};
        virtual void set_d_term(float) {};
        virtual void get_pid_settings(void) {};
        virtual void wait_for_spindle(void);
};

#endif
