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
    robot = new webots::Supervisor();

    // Camera
    camera = robot->getCamera("camera");
    if (camera) camera->enable(this->getPeriod(STATES::Compute)); else std::cout << "Cámara no encontrada." << std::endl;

    // Motors
    const char* motorNames[2] = {"motor_right", "motor_left"};
    for (size_t i = 0; i < motors.size(); ++i)
    {
        motors[i] = robot->getMotor(motorNames[i]);
        motors[i]->setPosition(INFINITY);                   // Velocity mode
        motors[i]->setVelocity(0);                      // Initial velocity
    }

    // Lidars
    for (int i=0; i<webotsLidars.size(); i++){
        std::string lidar_name = (i == 0) ? "lidar" : "lidar(" + std::to_string(i) + ")";
        webots::DistanceSensor* lidar = robot->getDistanceSensor(lidar_name);
        if (lidar){
            lidar->enable(this->getPeriod(STATES::Compute));
            webotsLidars[i] = lidar;
            robocompLidars.emplace_back();
        } else std::cout << "Lidar: " << lidar_name << " no encontrado." << std::endl;
    }

    robot->step(this->getPeriod(STATES::Compute));
}


void SpecificWorker::compute()
{
    receivingImageData();
    receivingLidarsData();
    
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

    printNotImplementedWarningMessage("DifferentialRobot_correctOdometer");
}

void SpecificWorker::DifferentialRobot_getBasePose(int &x, int &z, float &alpha)
{
#ifdef HIBERNATION_ENABLED
    hibernation = true;
#endif

    const double *pos = robot->getSelf()->getPosition();
    const double alpha_rad = robot->getSelf()->getOrientation()[2]; // Angulo de rotación en el plano

    x = static_cast<int>(pos[0] * 1000);  // Convertir metros a milímetros
    z = static_cast<int>(pos[2] * 1000);  // Convertir metros a milímetros
    alpha = static_cast<float>(alpha_rad);  // En radianes
}

void SpecificWorker::DifferentialRobot_getBaseState(RoboCompGenericBase::TBaseState &state)
{
#ifdef HIBERNATION_ENABLED
    hibernation = true;
#endif
    // Robot position
    const double *pos =  robot->getSelf()->getPosition();
    state.x = static_cast<int>(pos[0] * 1000);  // Convertir a milímetros
    state.z = static_cast<int>(pos[2] * 1000);  // Convertir a milímetros

    // Robot orientation
    state.alpha = robot->getSelf()->getOrientation()[2];  // Usar rotación en el plano

    // Robot speeds (can be obtained from motors)
    state.advVx = motors[0]->getVelocity();  // Velocidad del motor derecho
    state.rotV = motors[1]->getVelocity();  // Velocidad del motor izquierdo
}

void SpecificWorker::DifferentialRobot_resetOdometer()
{
#ifdef HIBERNATION_ENABLED
    hibernation = true;
#endif

    printNotImplementedWarningMessage("DifferentialRobot_resetOdometer");
}

void SpecificWorker::DifferentialRobot_setOdometer(RoboCompGenericBase::TBaseState state)
{
#ifdef HIBERNATION_ENABLED
    hibernation = true;
#endif

    printNotImplementedWarningMessage("DifferentialRobot_setOdometer");
}

void SpecificWorker::DifferentialRobot_setOdometerPose(int x, int z, float alpha)
{
#ifdef HIBERNATION_ENABLED
    hibernation = true;
#endif

    printNotImplementedWarningMessage("DifferentialRobot_setOdometerPose");
}

void SpecificWorker::DifferentialRobot_setSpeedBase(float adv, float rot)
{
#ifdef HIBERNATION_ENABLED
    hibernation = true;
#endif

    float right_speed = adv + rot;
    float left_speed = adv - rot;

    motors[0]->setVelocity(right_speed);
    motors[1]->setVelocity(left_speed);

    std::cout << "Setting speed: adv = " << adv << ", rot = " << rot << std::endl;
}

void SpecificWorker::DifferentialRobot_stopBase()
{
#ifdef HIBERNATION_ENABLED
    hibernation = true;
#endif

    motors[0]->setVelocity(0);
    motors[1]->setVelocity(0);

    std::cout << "Robot stopped." << std::endl;
}

#pragma endregion

#pragma region EmergencyStop

bool SpecificWorker::EmergencyStop_isEmergency()
{
#ifdef HIBERNATION_ENABLED
    hibernation = true;
#endif

    printNotImplementedWarningMessage("EmergencyStop_isEmergency");
    return false;
}

#pragma endregion

#pragma region EmotionalRobot
void SpecificWorker::EmotionalMotor_expressAnger()
{
#ifdef HIBERNATION_ENABLED
    hibernation = true;
#endif

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

    return robocompLidars;
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

void SpecificWorker::printNotImplementedWarningMessage(string functionName)
{
#ifdef HIBERNATION_ENABLED
    hibernation = true;
#endif

    cout << "Function not implemented used: " << "[" << functionName << "]" << std::endl;
}

void SpecificWorker::testMovement()
{
    // Variables para la posición inicial
    int initial_x = 0, initial_z = 0;
    float initial_alpha = 0.0;

    // Obtener la posición inicial del robot
    DifferentialRobot_getBasePose(initial_x, initial_z, initial_alpha);
    std::cout << "Starting position - x: " << initial_x << ", z: " << initial_z << ", alpha: " << initial_alpha << std::endl;

    // 1. Avanzar hacia adelante durante 2 segundos
    std::cout << "Moving forward..." << std::endl;
    DifferentialRobot_setSpeedBase(1.0, 0.0);  // Avanzar recto (velocidad de avance 1.0)
    robot->step(2000);  // Esperar 2 segundos
    printPosition();

    // 2. Girar a la derecha (rotación en sentido antihorario)
    std::cout << "Turning right..." << std::endl;
    DifferentialRobot_setSpeedBase(0.0, -0.5);  // Girar a la derecha (velocidad de rotación -0.5)
    robot->step(1000);  // Esperar 1 segundo
    printPosition();

    // 3. Avanzar hacia adelante durante 2 segundos
    std::cout << "Moving forward again..." << std::endl;
    DifferentialRobot_setSpeedBase(1.0, 0.0);
    robot->step(2000);
    printPosition();

    // 4. Moverse hacia atrás durante 2 segundos
    std::cout << "Moving backward..." << std::endl;
    DifferentialRobot_setSpeedBase(-1.0, 0.0);  // Ir marcha atrás
    robot->step(2000);
    printPosition();

    // 5. Girar 180 grados
    std::cout << "Turning..." << std::endl;
    DifferentialRobot_setSpeedBase(0.0, 1.0);  // Girar en sentido horario
    robot->step(2000);  // Girar durante 2 segundos
    printPosition();

    // 6. Avanzar un poco más hacia adelante durante 2 segundos
    std::cout << "Moving forward a bit more..." << std::endl;
    DifferentialRobot_setSpeedBase(1.0, 0.0);
    robot->step(2000);
    printPosition();

    // 7. Dar la vuelta y volver a la posición inicial
    std::cout << "Returning to the initial position..." << std::endl;
    DifferentialRobot_setSpeedBase(0.0, 1.0);  // Girar en sentido horario
    robot->step(2000);
    DifferentialRobot_setSpeedBase(-1.0, 0.0);  // Ir marcha atrás para volver
    robot->step(2000);
    printPosition();

    // 8. Detener el robot
    std::cout << "Stopping robot..." << std::endl;
    DifferentialRobot_stopBase();

    // Obtener la posición final y compararla con la inicial
    int final_x = 0, final_z = 0;
    float final_alpha = 0.0;
    DifferentialRobot_getBasePose(final_x, final_z, final_alpha);

    std::cout << "Final position - x: " << final_x << ", z: " << final_z << ", alpha: " << final_alpha << std::endl;
}

// Función auxiliar para imprimir la posición actual
void SpecificWorker::printPosition()
{
    int x = 0, z = 0;
    float alpha = 0.0;
    DifferentialRobot_getBasePose(x, z, alpha);
    std::cout << "Current position - x: " << x << ", z: " << z << ", alpha: " << alpha << std::endl;
}

void SpecificWorker::receivingLidarsData() {
    for(int i=0; i < webotsLidars.size(); i++){
        webots::DistanceSensor* lidar = webotsLidars[i];

        RoboCompLaser::TData robocompLidar;
        robocompLidar.angle = lidarAngles[i];
        robocompLidar.dist = static_cast<float>(lidar->getValue());
        robocompLidars[i] = robocompLidar;
    }
}

void SpecificWorker::printLidars() {
    for (int i = 0; i < robocompLidars.size(); ++i) {
        std::cout << "Lidar n." << i << ":" << std::endl;
        std::cout << "\tangle: " << robocompLidars[i].angle << std::endl;
        std::cout << "\tdist: " << robocompLidars[i].dist << std::endl;
    }
}




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

