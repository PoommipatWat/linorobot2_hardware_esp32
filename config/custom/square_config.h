// Copyright (c) 2021 Juan Miguel Jimeno
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef SQUARE_CONFIG_H
#define SQUARE_CONFIG_H

#define LED_PIN 13 

#define LINO_BASE DIFFERENTIAL_DRIVE 

#define USE_GENERIC_2_IN_MOTOR_DRIVER 
#define USE_GY85_IMU

#define K_P 20.                           
#define K_I 0.8                             
#define K_D 5.                             

#define MOTOR_MAX_RPM 100
#define MAX_RPM_RATIO 0.8
#define MOTOR_OPERATING_VOLTAGE 12
#define MOTOR_POWER_MAX_VOLTAGE 12
#define MOTOR_POWER_MEASURED_VOLTAGE 11.67                         
#define COUNTS_PER_REV1 2513
#define COUNTS_PER_REV2 2580
#define COUNTS_PER_REV3 2421
#define COUNTS_PER_REV4 2501
#define WHEEL_DIAMETER 0.09               
#define LR_WHEELS_DISTANCE 0.24            
#define PWM_BITS 10                        
#define PWM_FREQUENCY 20000

/// ENCODER PINS
#define MOTOR1_ENCODER_A 14
#define MOTOR1_ENCODER_B 15 
#define MOTOR1_ENCODER_INV false 

#define MOTOR2_ENCODER_A 11
#define MOTOR2_ENCODER_B 12 
#define MOTOR2_ENCODER_INV false 

#define MOTOR3_ENCODER_A 17
#define MOTOR3_ENCODER_B 16 
#define MOTOR3_ENCODER_INV true 

#define MOTOR4_ENCODER_A 9
#define MOTOR4_ENCODER_B 10
#define MOTOR4_ENCODER_INV false 

// Motor Pins
#define MOTOR1_PWM 1
#define MOTOR1_IN_A 20
#define MOTOR1_IN_B 21
#define MOTOR1_INV false

#define MOTOR2_PWM 5
#define MOTOR2_IN_A 6
#define MOTOR2_IN_B 8
#define MOTOR2_INV true

#define MOTOR3_PWM 22
#define MOTOR3_IN_A 23
#define MOTOR3_IN_B 0
#define MOTOR3_INV false

#define MOTOR4_PWM 4
#define MOTOR4_IN_A 3
#define MOTOR4_IN_B 2
#define MOTOR4_INV false

#define PWM_MAX pow(2, PWM_BITS) - 1
#define PWM_MIN -PWM_MAX

#endif
