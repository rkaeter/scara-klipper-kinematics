// SCARA kinematics stepper pulse time generation
// Copyright (C) 2025 R.Kaeter <kaeter@gmail.com>
// This file may be distributed under the terms of the GNU GPLv3 license.

#include <math.h> // sqrt, atan2, acos
#include <stddef.h> // offsetof
#include <stdlib.h> // malloc, free
#include <string.h> // memset
#include "compiler.h" // __visible
#include "itersolve.h" // struct stepper_kinematics
#include "trapq.h" // move_get_coord

struct scara_stepper {
    struct stepper_kinematics sk;
    double arm_a, arm_b;
};

// Inverse kinematics function for the first arm's stepper (Angle 1)
static double
scara_stepper_calc_angle1(struct stepper_kinematics *sk, struct move *m, double move_time)
{
    struct scara_stepper *ss = container_of(sk, struct scara_stepper, sk);
    struct coord c = move_get_coord(m, move_time);
    double x = c.x;
    double y = c.y;
    double r2 = x*x + y*y;
    double r = sqrt(r2);
    double alpha = atan2(y, x);
    double beta = acos((ss->arm_a * ss->arm_a + r2 - ss->arm_b * ss->arm_b) / (2 * ss->arm_a * r));
    double theta1 = alpha + beta;
    return theta1;
}

// Inverse kinematics function for the second arm's stepper (Angle 2)
static double
scara_stepper_calc_angle2(struct stepper_kinematics *sk, struct move *m, double move_time)
{
    struct scara_stepper *ss = container_of(sk, struct scara_stepper, sk);
    struct coord c = move_get_coord(m, move_time);
    double x = c.x;
    double y = c.y;
    double r2 = x*x + y*y;
    double theta2 = acos((r2 - ss->arm_a * ss->arm_a - ss->arm_b * ss->arm_b) / (2 * ss->arm_a * ss->arm_b));
    return theta2;
}

struct stepper_kinematics * __visible
scara_stepper_alloc(char angle, double arm_a, double arm_b)
{
    struct scara_stepper *ss = malloc(sizeof(*ss));
    if (!ss)
        return NULL;
    memset(ss, 0, sizeof(*ss));
    ss->arm_a = arm_a;
    ss->arm_b = arm_b;
    switch(angle) {
        case 'a':
            ss->sk.calc_position_cb = scara_stepper_calc_angle1;
            break;
        case 'b':
            ss->sk.calc_position_cb = scara_stepper_calc_angle2;
            break;
    }
    ss->sk.active_flags = AF_X | AF_Y;
    return &ss->sk;
}