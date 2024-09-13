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
#include "specificworker.h"

/**
* \brief Default constructor
*/
SpecificWorker::SpecificWorker(TuplePrx tprx, bool startup_check) : GenericWorker(tprx)
{
    this->startup_check_flag = startup_check;
}

/**
* \brief Default destructor
*/
SpecificWorker::~SpecificWorker()
{
    std::cout << "Destroying SpecificWorker" << std::endl;
}

bool SpecificWorker::setParams(RoboCompCommonBehavior::ParameterList params)
{
    return true;
}

void SpecificWorker::initialize()
{
    std::cout << "Initialize worker" << std::endl;
    if(this->startup_check_flag)
    {
        this->startup_check();
    }
    else
    {
#ifdef HIBERNATION_ENABLED
        hibernationChecker.start(500);
#endif
        initializeRobot();
    }

}

void SpecificWorker::initializeRobot(){
    robot = new webots::Robot();

    camera = robot->getCamera("camera");
    if (camera) camera->enable(this->getPeriod(STATES::Compute)); else std::cout << "Cámara no encontrada.";

    const char* motorNames[2] = {"motor_right", "motor_left"};
    for (size_t i = 0; i < motors.size(); ++i)
    {
        motors[i] = robot->getMotor(motorNames[i]);
        motors[i]->setPosition(INFINITY);                   // Velocity mode
        motors[i]->setVelocity(0);                      // Initial velocity
    }

    robot->step(this->getPeriod(STATES::Initialize));
}


void SpecificWorker::compute()
{
    receivingImageData();

    robot->step(this->getPeriod(STATES::Compute));
}

void SpecificWorker::emergency()
{
    std::cout << "Emergency worker" << std::endl;
    for(auto motor: motors) motor->setVelocity(0);
}

//Execute one when exiting to emergencyState
void SpecificWorker::restore()
{
    std::cout << "Restore worker" << std::endl;
}

int SpecificWorker::startup_check()
{
    std::cout << "Startup check" << std::endl;
    QTimer::singleShot(200, qApp, SLOT(quit()));
    return 0;
}

#pragma region Battery

RoboCompBatteryStatus::TBattery SpecificWorker::BatteryStatus_getBatteryState()
{
#ifdef HIBERNATION_ENABLED
    hibernation = true;
#endif
//implementCODE

}

#pragma endregion

#pragma region CameraSimple

RoboCompCameraSimple::TImage SpecificWorker::CameraSimple_getImage()
{
#ifdef HIBERNATION_ENABLED
    hibernation = true;
#endif
    return imgData;
}

void SpecificWorker::receivingImageData() {

    // Obtener el buffer de imagen
    const unsigned char* image = camera->getImage();
    if (!image) {
        throw std::runtime_error("Error al obtener la imagen de la cámara.");
    }

    // Configurar los campos de la estructura TImage
    imgData.compressed = false;  // Asumimos que la imagen no está comprimida
    imgData.width = camera->getWidth();
    imgData.height = camera->getHeight();
    imgData.depth = 3;  // Asumimos una imagen RGB con 3 bytes por píxel (8 bits por canal)

    // Crear un vector de bytes para almacenar los datos de la imagen
    const int totalPixels = imgData.width * imgData.height * imgData.depth;
    imgData.image.resize(totalPixels);

    // Copiar la imagen de la cámara a imgData.image (asumiendo formato RGB)
    for (int i = 0; i < totalPixels; i++) {
        imgData.image[i] = image[i];  // Copiamos el buffer crudo de la imagen
    }
}

#pragma endregion

#pragma region DifferentialRobot

void SpecificWorker::DifferentialRobot_correctOdometer(int x, int z, float alpha)
{
#ifdef HIBERNATION_ENABLED
    hibernation = true;
#endif
//implementCODE

}

void SpecificWorker::DifferentialRobot_getBasePose(int &x, int &z, float &alpha)
{
#ifdef HIBERNATION_ENABLED
    hibernation = true;
#endif
//implementCODE

}

void SpecificWorker::DifferentialRobot_getBaseState(RoboCompGenericBase::TBaseState &state)
{
#ifdef HIBERNATION_ENABLED
    hibernation = true;
#endif
//implementCODE

}

void SpecificWorker::DifferentialRobot_resetOdometer()
{
#ifdef HIBERNATION_ENABLED
    hibernation = true;
#endif
//implementCODE

}

void SpecificWorker::DifferentialRobot_setOdometer(RoboCompGenericBase::TBaseState state)
{
#ifdef HIBERNATION_ENABLED
    hibernation = true;
#endif
//implementCODE

}

void SpecificWorker::DifferentialRobot_setOdometerPose(int x, int z, float alpha)
{
#ifdef HIBERNATION_ENABLED
    hibernation = true;
#endif
//implementCODE

}

void SpecificWorker::DifferentialRobot_setSpeedBase(float adv, float rot)
{
#ifdef HIBERNATION_ENABLED
    hibernation = true;
#endif
//implementCODE

}

void SpecificWorker::DifferentialRobot_stopBase()
{
#ifdef HIBERNATION_ENABLED
    hibernation = true;
#endif
//implementCODE

}

#pragma endregion

#pragma region EmergencyStop

bool SpecificWorker::EmergencyStop_isEmergency()
{
#ifdef HIBERNATION_ENABLED
    hibernation = true;
#endif
//implementCODE

}

#pragma endregion

#pragma region EmotionalRobot
void SpecificWorker::EmotionalMotor_expressAnger()
{
#ifdef HIBERNATION_ENABLED
    hibernation = true;
#endif
//implementCODE

}

void SpecificWorker::EmotionalMotor_expressDisgust()
{
#ifdef HIBERNATION_ENABLED
    hibernation = true;
#endif
//implementCODE

}

void SpecificWorker::EmotionalMotor_expressFear()
{
#ifdef HIBERNATION_ENABLED
    hibernation = true;
#endif
//implementCODE

}

void SpecificWorker::EmotionalMotor_expressJoy()
{
#ifdef HIBERNATION_ENABLED
    hibernation = true;
#endif
//implementCODE

}

void SpecificWorker::EmotionalMotor_expressSadness()
{
#ifdef HIBERNATION_ENABLED
    hibernation = true;
#endif
//implementCODE

}

void SpecificWorker::EmotionalMotor_expressSurprise()
{
#ifdef HIBERNATION_ENABLED
    hibernation = true;
#endif
//implementCODE

}

void SpecificWorker::EmotionalMotor_isanybodythere(bool isAny)
{
#ifdef HIBERNATION_ENABLED
    hibernation = true;
#endif
//implementCODE

}

void SpecificWorker::EmotionalMotor_listening(bool setListening)
{
#ifdef HIBERNATION_ENABLED
    hibernation = true;
#endif
//implementCODE

}

void SpecificWorker::EmotionalMotor_pupposition(float x, float y)
{
#ifdef HIBERNATION_ENABLED
    hibernation = true;
#endif
//implementCODE

}

void SpecificWorker::EmotionalMotor_talking(bool setTalk)
{
#ifdef HIBERNATION_ENABLED
    hibernation = true;
#endif
//implementCODE

}

#pragma endregion

#pragma region Laser

RoboCompLaser::TLaserData SpecificWorker::Laser_getLaserAndBStateData(RoboCompGenericBase::TBaseState &bState)
{
#ifdef HIBERNATION_ENABLED
    hibernation = true;
#endif
//implementCODE

}

RoboCompLaser::LaserConfData SpecificWorker::Laser_getLaserConfData()
{
#ifdef HIBERNATION_ENABLED
    hibernation = true;
#endif
//implementCODE

}

RoboCompLaser::TLaserData SpecificWorker::Laser_getLaserData()
{
#ifdef HIBERNATION_ENABLED
    hibernation = true;
#endif
//implementCODE

}

#pragma endregion

#pragma region Speech
bool SpecificWorker::Speech_isBusy()
{
#ifdef HIBERNATION_ENABLED
    hibernation = true;
#endif
//implementCODE

}

bool SpecificWorker::Speech_say(std::string text, bool overwrite)
{
#ifdef HIBERNATION_ENABLED
    hibernation = true;
#endif
//implementCODE

}

#pragma endregion



/**************************************/
// From the RoboCompEmergencyStopPub you can publish calling this methods:
// this->emergencystoppub_pubproxy->emergencyStop(...)

/**************************************/
// From the RoboCompBatteryStatus you can use this types:
// RoboCompBatteryStatus::TBattery

/**************************************/
// From the RoboCompCameraSimple you can use this types:
// RoboCompCameraSimple::TImage

/**************************************/
// From the RoboCompDifferentialRobot you can use this types:
// RoboCompDifferentialRobot::TMechParams

/**************************************/
// From the RoboCompLaser you can use this types:
// RoboCompLaser::LaserConfData
// RoboCompLaser::TData

