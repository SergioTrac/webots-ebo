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

/**
	\brief
	@author authorname
*/



#ifndef SPECIFICWORKER_H
#define SPECIFICWORKER_H

#define HIBERNATION_ENABLED

#include <genericworker.h>

class SpecificWorker : public GenericWorker
{
Q_OBJECT
public:
	SpecificWorker(TuplePrx tprx, bool startup_check);
	~SpecificWorker();
	bool setParams(RoboCompCommonBehavior::ParameterList params);

	RoboCompBatteryStatus::TBattery BatteryStatus_getBatteryState();
	RoboCompCameraSimple::TImage CameraSimple_getImage();
	void DifferentialRobot_correctOdometer(int x, int z, float alpha);
	void DifferentialRobot_getBasePose(int &x, int &z, float &alpha);
	void DifferentialRobot_getBaseState(RoboCompGenericBase::TBaseState &state);
	void DifferentialRobot_resetOdometer();
	void DifferentialRobot_setOdometer(RoboCompGenericBase::TBaseState state);
	void DifferentialRobot_setOdometerPose(int x, int z, float alpha);
	void DifferentialRobot_setSpeedBase(float adv, float rot);
	void DifferentialRobot_stopBase();
	bool EmergencyStop_isEmergency();
	void EmotionalMotor_expressAnger();
	void EmotionalMotor_expressDisgust();
	void EmotionalMotor_expressFear();
	void EmotionalMotor_expressJoy();
	void EmotionalMotor_expressSadness();
	void EmotionalMotor_expressSurprise();
	void EmotionalMotor_isanybodythere(bool isAny);
	void EmotionalMotor_listening(bool setListening);
	void EmotionalMotor_pupposition(float x, float y);
	void EmotionalMotor_talking(bool setTalk);
	RoboCompLEDArray::PixelArray LEDArray_getLEDArray();
	RoboCompLEDArray::byte LEDArray_setLEDArray(RoboCompLEDArray::PixelArray pixelArray);
	RoboCompLaser::TLaserData Laser_getLaserAndBStateData(RoboCompGenericBase::TBaseState &bState);
	RoboCompLaser::LaserConfData Laser_getLaserConfData();
	RoboCompLaser::TLaserData Laser_getLaserData();
	float RGBSensor_getLux();
	RoboCompRGBSensor::RGBPixel RGBSensor_getRGBPixel();
	RoboCompRGBSensor::RGBPixelRAW RGBSensor_getRGBPixelRAW();
	float RGBSensor_getTemperature();
	void RGBSensor_setLight(RoboCompRGBSensor::byte percentageLight);
	bool Speech_isBusy();
	bool Speech_say(std::string text, bool overwrite);


public slots:
	void initialize();
	void compute();
	void emergency();
	void restore();
	int startup_check();
private:
	bool startup_check_flag;

};

#endif
