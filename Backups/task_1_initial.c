#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "ch.h"
#include "hal.h"
#include "memory_protection.h"
#include <main.h>
#include "leds.h"
#include "spi_comm.h"
#include "sensors/proximity.h"
#include "motors.h"
#include "sensors/VL53L0X/VL53L0X.h"

messagebus_t bus;
MUTEX_DECL(bus_lock);
CONDVAR_DECL(bus_condvar);

/*
Global Variables to describe the robots movement
*/
int object_position;  // Which side of the robot and object is. 1 for left, -1 for right.
int direction = 1;  // Direction of travel. 1 for forwards, -1 for backwards.

/*
Simple fucntion to set both motor speeds at once.
*/
void setSpeed(int speed)
{
    left_motor_set_speed(speed);
    right_motor_set_speed(speed);
}

/*
Function takes an input of an objects direction and will turn the robot away from that object.
Uses LED's to communicate turn direction with user.
*/
void turn(int object_position)
{
    if (object_position < 0) // Object on the right (turn right)
    {
        right_motor_set_speed(800);
        left_motor_set_speed(0);
        set_led(LED7, 1);
        chThdSleepMilliseconds(300);  // Required to prevent jittery behaviour and to cause LED's to flash
        set_led(LED7, 0);
    }
    else                   // Object on the left (turn left)
    {
        left_motor_set_speed(800);
        right_motor_set_speed(0);
        set_led(LED3, 1);
        chThdSleepMilliseconds(300);
        set_led(LED3, 0);
    }
}

/*
Function uses the forward (if going forwards) or rear (if going backwards) sensors to get the
distance between the robot and anything in the direction of travel of the robot.
*/
int findObject(void)
{
    int right_prox;
    int left_prox;
    int distance;

    if (direction == 1) // Forwards requires prox sensors 0 and 7. Backwards requires 3 and 4.
    {
        right_prox = 0;
        left_prox = 7;
    }
    else
    {
        right_prox = 4;
        left_prox = 3;
    }

    if (get_prox(right_prox) > get_prox(left_prox)) // Both sensors usually have different values, we want the closest value to be safe.
    {
        distance = get_prox(right_prox); // object is closest to the right side of the robot
        object_position = -1;             // -1 for right
    }
    else
    {
        distance = get_prox(left_prox); // object is closest to the left side of the robot
        object_position = 1;             // 1 For left
    }
    return(distance);
}

int main(void)
{

    halInit();
    chSysInit();
    mpu_init();

    // Proximity
    messagebus_init(&bus, &bus_lock, &bus_condvar);
    proximity_start(0);
    calibrate_ir();

    //LED
    clear_leds();
    spi_comm_start();

    //Motors
    motors_init();

    /* Infinite loop. */
    while (1)
    {
        setSpeed(direction * 1000);

        if ((findObject() > 40) && (findObject() < 800))  // If the robot senses a object too close it will attempt to turn away from it.
        {
            turn(object_position);
        }
        if (findObject() > 800) // Robot cannot always turn in time. If the robot is about to collide it will instead reverse directions.
        {
            direction = (direction * -1);
        }
    }
}

#define STACK_CHK_GUARD 0xe2dee396
uintptr_t __stack_chk_guard = STACK_CHK_GUARD;

void __stack_chk_fail(void)
{
    chSysHalt("Stack smashing detected");
}
