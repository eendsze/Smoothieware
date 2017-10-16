/*
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.
*/

#include "libs/Module.h"
#include "libs/Kernel.h"
#include "libs/SerialMessage.h"
#include <math.h>
#include "Switch.h"
#include "libs/Pin.h"
#include "modules/robot/Conveyor.h"
#include "PublicDataRequest.h"
#include "PublicData.h"
#include "SwitchPublicAccess.h"
#include "SlowTicker.h"
#include "Config.h"
#include "Gcode.h"
#include "checksumm.h"
#include "ConfigValue.h"
#include "StreamOutput.h"
#include "StreamOutputPool.h"
#include "PlayerPublicAccess.h"

#include "PwmOut.h"

#include "MRI_Hooks.h"

#include <algorithm>

#define    startup_state_checksum       CHECKSUM("startup_state")
#define    startup_value_checksum       CHECKSUM("startup_value")
#define    input_pin_checksum           CHECKSUM("input_pin")
#define    input_pin_behavior_checksum  CHECKSUM("input_pin_behavior")
#define    toggle_checksum              CHECKSUM("toggle")
#define    momentary_checksum           CHECKSUM("momentary")
#define    longpush_checksum            CHECKSUM("longpush")
#define    command_subcode_checksum     CHECKSUM("subcode")
#define    input_on_command_checksum    CHECKSUM("input_on_command")
#define    input_off_command_checksum   CHECKSUM("input_off_command")
#define    output_pin_checksum          CHECKSUM("output_pin")
#define    output_type_checksum         CHECKSUM("output_type")
#define    max_pwm_checksum             CHECKSUM("max_pwm")
#define    output_on_command_checksum   CHECKSUM("output_on_command")
#define    output_off_command_checksum  CHECKSUM("output_off_command")
#define    pwm_period_ms_checksum       CHECKSUM("pwm_period_ms")
#define    failsafe_checksum            CHECKSUM("failsafe_set_to")
#define    ignore_onhalt_checksum       CHECKSUM("ignore_on_halt")
#define    protected_action_checksum    CHECKSUM("protected") //switch works only if not playing and idle

Switch::Switch() {}

Switch::Switch(uint16_t name)
{
    name_checksum = name;
    //dummy_stream = &(StreamOutput::NullStream);
}

// set the pin to the fail safe value on halt
void Switch::on_halt(void *arg)
{
    if(arg == nullptr) {
        if(ignore_on_halt) return;

        // set pin to failsafe value
        switch(output_type) {
            case DIGITAL: digital_pin->set(failsafe); break;
            case SIGMADELTA: sigmadelta_pin->set(failsafe); break;
            case HWPWM: pwm_pin->write(0); break;
            case NONE: break;
        }
        switch_state= failsafe;
    }
}

void Switch::on_module_loaded()
{
    switch_changed = false;
    longpush_state = false;
    timer = 0;

    register_for_event(ON_GCODE_RECEIVED);
    register_for_event(ON_MAIN_LOOP);
    register_for_event(ON_GET_PUBLIC_DATA);
    register_for_event(ON_SET_PUBLIC_DATA);
    register_for_event(ON_HALT);

    // Settings
    on_config_reload(this);
}

// Get config
void Switch::on_config_reload(void *argument)
{
    input_pin.from_string( THEKERNEL->config->value(switch_checksum, name_checksum, input_pin_checksum )->by_default("nc")->as_string())->as_input();
    subcode = THEKERNEL->config->value(switch_checksum, name_checksum, command_subcode_checksum )->by_default(0)->as_number();
    std::string input_on_command = THEKERNEL->config->value(switch_checksum, name_checksum, input_on_command_checksum )->by_default("")->as_string();
    std::string input_off_command = THEKERNEL->config->value(switch_checksum, name_checksum, input_off_command_checksum )->by_default("")->as_string();
    output_on_command = THEKERNEL->config->value(switch_checksum, name_checksum, output_on_command_checksum )->by_default("")->as_string();
    output_off_command = THEKERNEL->config->value(switch_checksum, name_checksum, output_off_command_checksum )->by_default("")->as_string();
    switch_state = THEKERNEL->config->value(switch_checksum, name_checksum, startup_state_checksum )->by_default(false)->as_bool();
    string type = THEKERNEL->config->value(switch_checksum, name_checksum, output_type_checksum )->by_default("pwm")->as_string();
    failsafe= THEKERNEL->config->value(switch_checksum, name_checksum, failsafe_checksum )->by_default(0)->as_number();
    ignore_on_halt= THEKERNEL->config->value(switch_checksum, name_checksum, ignore_onhalt_checksum )->by_default(false)->as_bool();
    protected_action= THEKERNEL->config->value(switch_checksum, name_checksum, protected_action_checksum )->by_default(false)->as_bool();

    std::string ipb = THEKERNEL->config->value(switch_checksum, name_checksum, input_pin_behavior_checksum )->by_default("momentary")->as_string();
    if(ipb == "toggle") {
        input_pin_behavior = toggle_checksum;
    } else if(ipb == "longpush") {
        input_pin_behavior = longpush_checksum;
    } else input_pin_behavior = momentary_checksum;

    if(type == "pwm"){
        output_type= SIGMADELTA;
        sigmadelta_pin= new Pwm();
        sigmadelta_pin->from_string(THEKERNEL->config->value(switch_checksum, name_checksum, output_pin_checksum )->by_default("nc")->as_string())->as_output();
        if(sigmadelta_pin->connected()) {
            if(failsafe == 1) {
                set_high_on_debug(sigmadelta_pin->port_number, sigmadelta_pin->pin);
            }else{
                set_low_on_debug(sigmadelta_pin->port_number, sigmadelta_pin->pin);
            }
        }else{
            output_type= NONE;
            delete sigmadelta_pin;
            sigmadelta_pin= nullptr;
        }

    }else if(type == "digital"){
        output_type= DIGITAL;
        digital_pin= new Pin();
        digital_pin->from_string(THEKERNEL->config->value(switch_checksum, name_checksum, output_pin_checksum )->by_default("nc")->as_string())->as_output();
        if(digital_pin->connected()) {
            if(failsafe == 1) {
                set_high_on_debug(digital_pin->port_number, digital_pin->pin);
            }else{
                set_low_on_debug(digital_pin->port_number, digital_pin->pin);
            }
        }else{
            output_type= NONE;
            delete digital_pin;
            digital_pin= nullptr;
        }

    }else if(type == "hwpwm"){
        output_type= HWPWM;
        Pin *pin= new Pin();
        pin->from_string(THEKERNEL->config->value(switch_checksum, name_checksum, output_pin_checksum )->by_default("nc")->as_string())->as_output();
        pwm_pin= pin->hardware_pwm();
        if(failsafe == 1) {
            set_high_on_debug(pin->port_number, pin->pin);
        }else{
            set_low_on_debug(pin->port_number, pin->pin);
        }
        delete pin;
        if(pwm_pin == nullptr) {
            THEKERNEL->streams->printf("Selected Switch output pin is not PWM capable - disabled");
            output_type= NONE;
        }

    } else {
        output_type= NONE;
    }

    if(output_type == SIGMADELTA) {
        sigmadelta_pin->max_pwm(THEKERNEL->config->value(switch_checksum, name_checksum, max_pwm_checksum )->by_default(255)->as_number());
        switch_value = THEKERNEL->config->value(switch_checksum, name_checksum, startup_value_checksum )->by_default(sigmadelta_pin->max_pwm())->as_number();
        if(switch_state) {
            sigmadelta_pin->pwm(switch_value); // will be truncated to max_pwm
        } else {
            sigmadelta_pin->set(false);
        }

    } else if(output_type == HWPWM) {
        // default is 50Hz
        float p= THEKERNEL->config->value(switch_checksum, name_checksum, pwm_period_ms_checksum )->by_default(20)->as_number() * 1000.0F; // ms but fractions are allowed
        pwm_pin->period_us(p);

        // default is 0% duty cycle
        switch_value = THEKERNEL->config->value(switch_checksum, name_checksum, startup_value_checksum )->by_default(0)->as_number();
        if(switch_state) {
            pwm_pin->write(switch_value/100.0F);
        } else {
            pwm_pin->write(0);
        }

    } else if(output_type == DIGITAL){
        digital_pin->set(switch_state);
    }

    // Set the on/off command codes, Use GCode to do the parsing
    input_on_command_letter = 0;
    input_off_command_letter = 0;

    if(!input_on_command.empty()) {
        Gcode gc(input_on_command, NULL);
        if(gc.has_g) {
            input_on_command_letter = 'G';
            input_on_command_code = gc.g;
        } else if(gc.has_m) {
            input_on_command_letter = 'M';
            input_on_command_code = gc.m;
        }
    }
    if(!input_off_command.empty()) {
        Gcode gc(input_off_command, NULL);
        if(gc.has_g) {
            input_off_command_letter = 'G';
            input_off_command_code = gc.g;
        } else if(gc.has_m) {
            input_off_command_letter = 'M';
            input_off_command_code = gc.m;
        }
    }

    if(input_pin.connected()) {
        // set to initial state
        input_pin_state = input_pin.get();
        // input pin polling
        THEKERNEL->slow_ticker->attach( 100, this, &Switch::pinpoll_tick);
    }

    if(output_type == SIGMADELTA) {
        // SIGMADELTA
        THEKERNEL->slow_ticker->attach(1000, sigmadelta_pin, &Pwm::on_tick);
    }

    // for commands we need to replace _ for space
    std::replace(output_on_command.begin(), output_on_command.end(), '_', ' '); // replace _ with space
    std::replace(output_off_command.begin(), output_off_command.end(), '_', ' '); // replace _ with space
}

bool Switch::match_input_on_gcode(const Gcode *gcode) const
{
    bool b= ((input_on_command_letter == 'M' && gcode->has_m && gcode->m == input_on_command_code) ||
            (input_on_command_letter == 'G' && gcode->has_g && gcode->g == input_on_command_code));

    return (b && gcode->subcode == subcode);
}

bool Switch::match_input_off_gcode(const Gcode *gcode) const
{
    bool b= ((input_off_command_letter == 'M' && gcode->has_m && gcode->m == input_off_command_code) ||
            (input_off_command_letter == 'G' && gcode->has_g && gcode->g == input_off_command_code));
    return (b && gcode->subcode == subcode);
}

void Switch::on_gcode_received(void *argument)
{
    Gcode *gcode = static_cast<Gcode *>(argument);
    // Add the gcode to the queue ourselves if we need it
    if (!(match_input_on_gcode(gcode) || match_input_off_gcode(gcode))) {
        return;
    }

    // we need to sync this with the queue, so we need to wait for queue to empty, however due to certain slicers
    // issuing redundant swicth on calls regularly we need to optimize by making sure the value is actually changing
    // hence we need to do the wait for queue in each case rather than just once at the start
    if(match_input_on_gcode(gcode)) {
    	switch_state = true; //this is for input switches, to be able to change it's state with G-code
        if (output_type == SIGMADELTA) {
            // SIGMADELTA output pin turn on (or off if S0)
            if(gcode->has_letter('S')) {
                int v = roundf(gcode->get_value('S') * sigmadelta_pin->max_pwm() / 255.0F); // scale by max_pwm so input of 255 and max_pwm of 128 would set value to 128
                if(v != sigmadelta_pin->get_pwm()){ // optimize... ignore if already set to the same pwm
                    // drain queue
                    THEKERNEL->conveyor->wait_for_idle();
                    sigmadelta_pin->pwm(v);
                    switch_state= (v > 0);
                }
            } else {
                // drain queue
                THEKERNEL->conveyor->wait_for_idle();
                sigmadelta_pin->pwm(switch_value);
                switch_state= (switch_value > 0);
            }

        } else if (output_type == HWPWM) {
            // drain queue
            THEKERNEL->conveyor->wait_for_idle();
            // PWM output pin set duty cycle 0 - 100
            if(gcode->has_letter('S')) {
                float v = gcode->get_value('S');
                if(v > 100) v= 100;
                else if(v < 0) v= 0;
                pwm_pin->write(v/100.0F);
                switch_state= (v != 0);
            } else {
                pwm_pin->write(switch_value);
                switch_state= (switch_value != 0);
            }

        } else if (output_type == DIGITAL) {
            // drain queue
            THEKERNEL->conveyor->wait_for_idle();
            // logic pin turn on
            digital_pin->set(true);
            switch_state = true;
        }

    } else if(match_input_off_gcode(gcode)) {
        // drain queue
        THEKERNEL->conveyor->wait_for_idle();
        switch_state = false;
        if (output_type == SIGMADELTA) {
            // SIGMADELTA output pin
            sigmadelta_pin->set(false);

        } else if (output_type == HWPWM) {
            pwm_pin->write(0);

        } else if (output_type == DIGITAL) {
            // logic pin turn off
            digital_pin->set(false);
        }
    }
}

void Switch::on_get_public_data(void *argument)
{
    PublicDataRequest *pdr = static_cast<PublicDataRequest *>(argument);

    if(!pdr->starts_with(switch_checksum)) return;

    if(!pdr->second_element_is(name_checksum)) return; // likely fan, but could be anything

    // ok this is targeted at us, so send back the requested data
    // caller has provided the location to write the state to
    struct pad_switch *pad= static_cast<struct pad_switch *>(pdr->get_data_ptr());
    pad->name = name_checksum;
    pad->state = switch_state;
    pad->value = switch_value;
    pdr->set_taken();
}

void Switch::on_set_public_data(void *argument)
{
    PublicDataRequest *pdr = static_cast<PublicDataRequest *>(argument);

    if(!pdr->starts_with(switch_checksum)) return;

    if(!pdr->second_element_is(name_checksum)) return; // likely fan, but could be anything

    // ok this is targeted at us, so set the value
    if(pdr->third_element_is(state_checksum)) {
        bool t = *static_cast<bool *>(pdr->get_data_ptr());
        switch_state = t;
        switch_changed= true;
        pdr->set_taken();

        // if there is no gcode to be sent then we can do this now (in on_idle)
        // Allows temperature switch to turn on a fan even if main loop is blocked with heat and wait
        if(output_on_command.empty() && output_off_command.empty()) on_main_loop(nullptr);

    } else if(pdr->third_element_is(value_checksum)) {
        float t = *static_cast<float *>(pdr->get_data_ptr());
        switch_value = t;
        switch_changed= true;
        pdr->set_taken();
    }
}

void Switch::on_main_loop(void *argument)
{
    bool do_switch_on_action = false;
    bool do_switch_off_action = false;

    if(switch_changed) {
        if(protected_action) {
            // in this mode we don't process the switch if not idle, or playin a file
            void *returned_data;
            bool playing = false;

            bool ok = PublicData::get_value( player_checksum, is_playing_checksum, &returned_data );
            if (ok) {
                playing = *static_cast<bool *>(returned_data);
            }

            if(!THECONVEYOR->is_idle() || playing) {
                switch_changed = false;
                return;
            }
        }

        if( input_pin_behavior != longpush_checksum) { //normal behavior
            if(switch_state) {
                do_switch_on_action = true;
            } else {
                do_switch_off_action = true;
            }
        } else { //wait to decide if long push
            if(switch_state) { // if button pushed
                timer = 0;
                longpush_state = true;
            } else {
                if(longpush_state) {
                    // it was a short push, handle it as an "on" action
                    do_switch_on_action = true;
                    longpush_state = false;
                }
            }
        }

        switch_changed = false;
    }

    if(longpush_state && timer >= 100) {
        //if timer elapsed, it is a long push; do "off" action
        do_switch_off_action = true;
        longpush_state = false;
    }

    {
        if(do_switch_on_action) {
            if(!output_on_command.empty()) send_gcode( output_on_command, &(StreamOutput::NullStream) );

            if(output_type == SIGMADELTA) {
                sigmadelta_pin->pwm(switch_value); // this requires the value has been set otherwise it switches on to whatever it last was

            } else if (output_type == HWPWM) {
                pwm_pin->write(switch_value/100.0F);

            } else if (output_type == DIGITAL) {
                digital_pin->set(true);
            }

        }
        if(do_switch_off_action) {

            if(!output_off_command.empty()) send_gcode( output_off_command, &(StreamOutput::NullStream) );

            if(output_type == SIGMADELTA) {
                sigmadelta_pin->set(false);

            } else if (output_type == HWPWM) {
                pwm_pin->write(0);

            } else if (output_type == DIGITAL) {
                digital_pin->set(false);
            }
        }
    }
}

// TODO Make this use InterruptIn
// Check the state of the button and act accordingly
uint32_t Switch::pinpoll_tick(uint32_t dummy)
{
    if(!input_pin.connected()) return 0;

    if(longpush_state) timer++;

    // If pin changed
    bool current_state = input_pin.get();
    if(input_pin_state != current_state) {
        input_pin_state = current_state;
        // If pin high
        if( input_pin_state ) {
            if( input_pin_behavior == toggle_checksum ) {
                flip();
            } else {
                // else default is momentary
                switch_state = input_pin_state;
                switch_changed = true;
            }

        } else {
            // else if button released
            if( input_pin_behavior != toggle_checksum ) {
                switch_state = input_pin_state;
                switch_changed = true;
            }
        }
    }
    return 0;
}

void Switch::flip()
{
    switch_state = !switch_state;
    switch_changed = true;
}

void Switch::send_gcode(std::string msg, StreamOutput *stream)
{
    struct SerialMessage message;
    message.message = msg;
    message.stream = stream;
    THEKERNEL->call_event(ON_CONSOLE_LINE_RECEIVED, &message );
}

