/**
 * COPYRIGHT and PERMISSION NOTICE
 * Real Deal Robotics Software: Utilities.h
 * Copyright (C) 2019 Real Deal Robotics, LLC
 * All rights Reserved
 */
 
#ifndef RDR_UTILITIES_H
#define RDR_UTILITIES_H

#include <cmath>

#ifndef RAD2DEG
#define RAD2DEG 180.0/M_PI
#endif

#ifndef DEG2RAD 
#define DEG2RAD M_PI/180.0
#endif

static constexpr double Gravity{9.80665};   // m/s
static constexpr double _max_accel{5 * Gravity};   // 5 G's


#endif // RDR_UTILITIES_H
