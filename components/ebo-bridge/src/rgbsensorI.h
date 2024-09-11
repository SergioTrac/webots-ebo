/*
 *    Copyright (C) 2024 by YOUR NAME HERE
 *
 *    This file is part of RoboComp
 *
 *    RoboComp is free software: you can redistribute it and/or modify
 *    it under the terms of the GNU General Public License as published by
 *    the Free Software Foundation, either version 3 of the License, or
 *    (at your option) any later version.
 *
 *    RoboComp is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU General Public License for more details.
 *
 *    You should have received a copy of the GNU General Public License
 *    along with RoboComp.  If not, see <http://www.gnu.org/licenses/>.
 */
#ifndef RGBSENSOR_H
#define RGBSENSOR_H

// Ice includes
#include <Ice/Ice.h>
#include <RGBSensor.h>

#include <config.h>
#include "genericworker.h"


class RGBSensorI : public virtual RoboCompRGBSensor::RGBSensor
{
public:
	RGBSensorI(GenericWorker *_worker);
	~RGBSensorI();

	float getLux(const Ice::Current&);
	RoboCompRGBSensor::RGBPixel getRGBPixel(const Ice::Current&);
	RoboCompRGBSensor::RGBPixelRAW getRGBPixelRAW(const Ice::Current&);
	float getTemperature(const Ice::Current&);
	void setLight(RoboCompRGBSensor::byte percentageLight, const Ice::Current&);

private:

	GenericWorker *worker;

};

#endif
