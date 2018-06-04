/**
 * \file    ev3cxx_infrared_sensor.h
 * \brief	EV3RT CPP API for infrared sensor
 * \author	Jaroslav Páral (jarekparal), Martin Mikšík
 */

#pragma once

#include <stdio.h>

#include "ev3api.h"
#include "ev3cxx_sensor.h"

namespace ev3cxx {
    
/**
 * \~English
 * \brief    Class InfraredSensor. API for working with infra sensor.
 */
class InfraredSensor : public Sensor
{
public:
    /**
     * \~English
     * \brief       Constructor of class InfraSensor.
     * \param port  Port index (1-4).      
     */  
    InfraredSensor(SensorPort port) : Sensor(port, INFRARED_SENSOR) {
        distance(); // By read one value immediate activate the sensor
    }

    /**
     * \~English
     * \brief       Get distance in centimeters. 
     * \return      Distance in cm. Range: 3 - 255 (too near/far => 255)      
     */  
    int distance() {
        return ev3_infrared_sensor_get_distance(m_port);
    }

    /**
     * \~English
     * \brief       Get millimeters in centimeters. 
     * \return      Distance in mm. Range: 30 - 2550 (too near/far => 2550)      
     */  
    int millimeters() {
        return ev3_ultrasonic_sensor_get_raw_data(m_port, US_DIST_METRIC);
    }

    /**
     * \~English
     * \brief       Get measure distance in inches. 
     * \return      Distance in in. Range: 1 - 100 (too near/far => 100)      
     */  
    int inches() {
        return ev3_ultrasonic_sensor_get_raw_data(m_port, US_DIST_IMPERIAL) / 10;
    }

    /**
     * \~English
     * \brief       Get measure distance in line (1/12 inch). 
     * \return      Distance in ln. Range: 13 - 1200 (too near/far => 1200)      
     */  
    int inchesLine() {
        return (ev3_ultrasonic_sensor_get_raw_data(m_port, US_DIST_IMPERIAL) * 12) / 10;
    }


    /**
     * \~English
     * \brief       Listen and return \a true if catch ultrasonic signal. 
     * \return      Return \a true if ultrasonic signal get, else \a false.    
     */  
    bool_t listen() {
        return ev3_ultrasonic_sensor_listen(m_port);
    }

}; // class InfraredSensor


} // namespace ev3cxx

