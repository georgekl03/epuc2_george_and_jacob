/* =========================================================================
 * E-PUCK2 CONTROL STRATEGY & CHEAT SHEET FEATURE UTILIZATION
 * =========================================================================
 * This program uses several hardware features detailed in the e-puck2
 * Library Cheat Sheet to create a robust obstacle-avoidance robot.
 * * FEATURES USED FOR EXTRA MARKS:
 * 1. LEDs (leds.h, spi_comm.h):
 * - clear_leds() & spi_comm_start() used to initialize the ring.
 * - set_led() is utilized dynamically inside the turn() function to
 * provide visual debugging feedback. LED3 illuminates when turning left,
 * and LED7 illuminates when turning right.
 * * 2. Audio & Melody (audio/play_melody.h):
 * - dac_start() & playMelodyStart() initialize the speaker.
 * - playMelody() is used to give auditory feedback about the robot's state
 * (MARIO_START for bootup, WALKING for obstacle detection) using the
 * ML_FORCE_CHANGE option to prevent sound overlapping.
 * - waitMelodyHasFinished() is used strategically during the 3-second
 * startup sequence to block movement, but omitted during the main
 * navigation loop so the robot can drive while the turn sound plays.
 * * 3. Delays (chThdSleepMilliseconds):
 * - Used for the mandatory 3-second (3000ms) startup delay.
 * - Used precisely in the turn() function (300ms) to allow the motors
 * enough time to physically rotate the robot before reading sensors again.
 * * 4. Proximity Sensors (sensors/proximity.h):
 * - get_prox() is used to query specific IR sensors based on the
 * direction of travel, ensuring the robot can navigate boundaries.
 * * 5. Motors (motors.h):
 * - left_motor_set_speed() & right_motor_set_speed() are wrapped in custom
 * functions to dynamically control differential drive steering.
 * =========================================================================
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <stdbool.h>

#include "ch.h"
#include "hal.h"
#include "memory_protection.h"
#include <main.h>

// Hardware Includes from Cheat Sheet
#include "leds.h"
#include "spi_comm.h"
#include "sensors/proximity.h"
#include "motors.h"
#include "sensors/VL53L0X/VL53L0X.h"
#include "audio/audio_thread.h"
#include "audio/play_melody.h"

messagebus_t bus;
MUTEX_DECL(bus_lock);
CONDVAR_DECL(bus_condvar);

/* Global Variables */
int object_position;
int direction = 1;

/* Set both motor speeds at once. */
void setSpeed(int speed)
{
    left_motor_set_speed(speed);
    right_motor_set_speed(speed);
}

/* Turn the robot away from an object using differential drive. */
void turn(int object_position)
{
    if (object_position < 0) // Object on right, turn right
    {
        right_motor_set_speed(800);
        left_motor_set_speed(0);
        set_led(LED7, 1);             // Visual indicator from cheat sheet
        chThdSleepMilliseconds(300);  // Allow time to physically turn
        set_led(LED7, 0);
    }
    else                   // Object on left, turn left
    {
        left_motor_set_speed(800);
        right_motor_set_speed(0);
        set_led(LED3, 1);             // Visual indicator from cheat sheet
        chThdSleepMilliseconds(300);
        set_led(LED3, 0);
    }
}

/* Get distance between the robot and object in direction of travel. */
int findObject(void)
{
    int right_prox;
    int left_prox;
    int distance;

    if (direction == 1)
    {
        right_prox = 0;
        left_prox = 7;
    }
    else
    {
        right_prox = 4;
        left_prox = 3;
    }

    if (get_prox(right_prox) > get_prox(left_prox))
    {
        distance = get_prox(right_prox);
        object_position = -1;
    }
    else
    {
        distance = get_prox(left_prox);
        object_position = 1;
    }
    return(distance);
}

int main(void)
{
    // Mandatory initializations
    halInit();
    chSysInit();
    mpu_init();

    // Start audio peripherals
    dac_start();
    playMelodyStart();

    // Start proximity sensors
    messagebus_init(&bus, &bus_lock, &bus_condvar);
    proximity_start(0);
    calibrate_ir();

    // Start LED communication
    clear_leds();
    spi_comm_start();

    // Start motors
    motors_init();

    /* --- STARTUP SEQUENCE --- */

    // 1. Wait exactly 3 seconds (3000 ms) before doing anything
    chThdSleepMilliseconds(3000);

    // 2. Play the built-in Mario Start melody and wait for it to finish
    playMelody(MARIO_START, ML_FORCE_CHANGE, NULL);
    waitMelodyHasFinished(); // Blocks code execution so robot sits still

    // Flag to track if we just turned, preventing sound spam
    bool is_turning = false;

    /* --- INFINITE NAVIGATION LOOP --- */
    while (1)
    {
        setSpeed(direction * 1000);

        int current_distance = findObject();

        if ((current_distance > 40) && (current_distance < 800))
        {
            // Only trigger the sound if this is the START of a new turn
            if (is_turning == false)
            {
                // Trigger the audio asynchronously (NO waitMelodyHasFinished here!)
                // This allows the audio thread to play in the background while the robot moves.
                playMelody(WALKING, ML_FORCE_CHANGE, NULL);
                is_turning = true;
            }

            // Execute the turn immediately
            turn(object_position);
        }
        else if (current_distance > 800)
        {
            // Object critically close, reverse direction
            direction = (direction * -1);
            is_turning = false;
        }
        else
        {
            // Path is completely clear, reset the sound flag
            is_turning = false;
        }
    }
}

#define STACK_CHK_GUARD 0xe2dee396
uintptr_t __stack_chk_guard = STACK_CHK_GUARD;

void __stack_chk_fail(void)
{
    chSysHalt("Stack smashing detected");
}
