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


/** \mainpage RoboComp::EboBridge
 *
 * \section intro_sec Introduction
 *
 * The EboBridge component...
 *
 * \section interface_sec Interface
 *
 * interface...
 *
 * \section install_sec Installation
 *
 * \subsection install1_ssec Software depencences
 * ...
 *
 * \subsection install2_ssec Compile and install
 * cd EboBridge
 * <br>
 * cmake . && make
 * <br>
 * To install:
 * <br>
 * sudo make install
 *
 * \section guide_sec User guide
 *
 * \subsection config_ssec Configuration file
 *
 * <p>
 * The configuration file etc/config...
 * </p>
 *
 * \subsection execution_ssec Execution
 *
 * Just: "${PATH_TO_BINARY}/EboBridge --Ice.Config=${PATH_TO_CONFIG_FILE}"
 *
 * \subsection running_ssec Once running
 *
 * ...
 *
 */
#include <signal.h>

// QT includes
#include <QtCore>
#include <QtWidgets>

// ICE includes
#include <Ice/Ice.h>
#include <IceStorm/IceStorm.h>
#include <Ice/Application.h>

#include <rapplication/rapplication.h>
#include <sigwatch/sigwatch.h>
#include <qlog/qlog.h>

#include "config.h"
#include "genericmonitor.h"
#include "genericworker.h"
#include "specificworker.h"
#include "specificmonitor.h"
#include "commonbehaviorI.h"

#include <batterystatusI.h>
#include <camerasimpleI.h>
#include <differentialrobotI.h>
#include <emergencystopI.h>
#include <emotionalmotorI.h>
#include <ledarrayI.h>
#include <laserI.h>
#include <speechI.h>

#include <GenericBase.h>
#include <GenericBase.h>



class EboBridge : public RoboComp::Application
{
public:
	EboBridge (QString prfx, bool startup_check) { prefix = prfx.toStdString(); this->startup_check_flag=startup_check; }
private:
	void initialize();
	std::string prefix;
	TuplePrx tprx;
	bool startup_check_flag = false;

public:
	virtual int run(int, char*[]);
};

void ::EboBridge::initialize()
{
	// Config file properties read example
	// configGetString( PROPERTY_NAME_1, property1_holder, PROPERTY_1_DEFAULT_VALUE );
	// configGetInt( PROPERTY_NAME_2, property1_holder, PROPERTY_2_DEFAULT_VALUE );
}

int ::EboBridge::run(int argc, char* argv[])
{
#ifdef USE_QTGUI
	QApplication a(argc, argv);  // GUI application
#else
	QCoreApplication a(argc, argv);  // NON-GUI application
#endif


	sigset_t sigs;
	sigemptyset(&sigs);
	sigaddset(&sigs, SIGHUP);
	sigaddset(&sigs, SIGINT);
	sigaddset(&sigs, SIGTERM);
	sigprocmask(SIG_UNBLOCK, &sigs, 0);

	UnixSignalWatcher sigwatch;
	sigwatch.watchForSignal(SIGINT);
	sigwatch.watchForSignal(SIGTERM);
	QObject::connect(&sigwatch, SIGNAL(unixSignal(int)), &a, SLOT(quit()));

	int status=EXIT_SUCCESS;

	RoboCompEmergencyStopPub::EmergencyStopPubPrxPtr emergencystoppub_pubproxy;

	string proxy, tmp;
	initialize();

	IceStorm::TopicManagerPrxPtr topicManager;
	try
	{
		topicManager = topicManager = Ice::checkedCast<IceStorm::TopicManagerPrx>(communicator()->propertyToProxy("TopicManager.Proxy"));
		if (!topicManager)
		{
		    cout << "[" << PROGRAM_NAME << "]: TopicManager.Proxy not defined in config file."<<endl;
		    cout << "	 Config line example: TopicManager.Proxy=IceStorm/TopicManager:default -p 9999"<<endl;
	        return EXIT_FAILURE;
		}
	}
	catch (const Ice::Exception &ex)
	{
		cout << "[" << PROGRAM_NAME << "]: Exception: 'rcnode' not running: " << ex << endl;
		return EXIT_FAILURE;
	}
	std::shared_ptr<IceStorm::TopicPrx> emergencystoppub_topic;

	while (!emergencystoppub_topic)
	{
		try
		{
			emergencystoppub_topic = topicManager->retrieve("EmergencyStopPub");
		}
		catch (const IceStorm::NoSuchTopic&)
		{
			cout << "[" << PROGRAM_NAME << "]: ERROR retrieving EmergencyStopPub topic. \n";
			try
			{
				emergencystoppub_topic = topicManager->create("EmergencyStopPub");
			}
			catch (const IceStorm::TopicExists&){
				// Another client created the topic.
				cout << "[" << PROGRAM_NAME << "]: ERROR publishing the EmergencyStopPub topic. It's possible that other component have created\n";
			}
		}
		catch(const IceUtil::NullHandleException&)
		{
			cout << "[" << PROGRAM_NAME << "]: ERROR TopicManager is Null. Check that your configuration file contains an entry like:\n"<<
			"\t\tTopicManager.Proxy=IceStorm/TopicManager:default -p <port>\n";
			return EXIT_FAILURE;
		}
	}

	auto emergencystoppub_pub = emergencystoppub_topic->getPublisher()->ice_oneway();
	emergencystoppub_pubproxy = Ice::uncheckedCast<RoboCompEmergencyStopPub::EmergencyStopPubPrx>(emergencystoppub_pub);

	tprx = std::make_tuple(emergencystoppub_pubproxy);
	SpecificWorker *worker = new SpecificWorker(tprx, startup_check_flag);
	//Monitor thread
	SpecificMonitor *monitor = new SpecificMonitor(worker,communicator());
	QObject::connect(monitor, SIGNAL(kill()), &a, SLOT(quit()));
	QObject::connect(worker, SIGNAL(kill()), &a, SLOT(quit()));
	monitor->start();

	if ( !monitor->isRunning() )
		return status;

	while (!monitor->ready)
	{
		usleep(10000);
	}

	try
	{
		try {
			// Server adapter creation and publication
			if (not GenericMonitor::configGetString(communicator(), prefix, "CommonBehavior.Endpoints", tmp, "")) {
				cout << "[" << PROGRAM_NAME << "]: Can't read configuration for proxy CommonBehavior\n";
			}
			Ice::ObjectAdapterPtr adapterCommonBehavior = communicator()->createObjectAdapterWithEndpoints("commonbehavior", tmp);
			auto commonbehaviorI = std::make_shared<CommonBehaviorI>(monitor);
			adapterCommonBehavior->add(commonbehaviorI, Ice::stringToIdentity("commonbehavior"));
			adapterCommonBehavior->activate();
		}
		catch(const Ice::Exception& ex)
		{
			status = EXIT_FAILURE;

			cout << "[" << PROGRAM_NAME << "]: Exception raised while creating CommonBehavior adapter: " << endl;
			cout << ex;

		}



		try
		{
			// Server adapter creation and publication
			if (not GenericMonitor::configGetString(communicator(), prefix, "BatteryStatus.Endpoints", tmp, ""))
			{
				cout << "[" << PROGRAM_NAME << "]: Can't read configuration for proxy BatteryStatus";
			}
			Ice::ObjectAdapterPtr adapterBatteryStatus = communicator()->createObjectAdapterWithEndpoints("BatteryStatus", tmp);
			auto batterystatus = std::make_shared<BatteryStatusI>(worker);
			adapterBatteryStatus->add(batterystatus, Ice::stringToIdentity("batterystatus"));
			adapterBatteryStatus->activate();
			cout << "[" << PROGRAM_NAME << "]: BatteryStatus adapter created in port " << tmp << endl;
		}
		catch (const IceStorm::TopicExists&){
			cout << "[" << PROGRAM_NAME << "]: ERROR creating or activating adapter for BatteryStatus\n";
		}


		try
		{
			// Server adapter creation and publication
			if (not GenericMonitor::configGetString(communicator(), prefix, "CameraSimple.Endpoints", tmp, ""))
			{
				cout << "[" << PROGRAM_NAME << "]: Can't read configuration for proxy CameraSimple";
			}
			Ice::ObjectAdapterPtr adapterCameraSimple = communicator()->createObjectAdapterWithEndpoints("CameraSimple", tmp);
			auto camerasimple = std::make_shared<CameraSimpleI>(worker);
			adapterCameraSimple->add(camerasimple, Ice::stringToIdentity("camerasimple"));
			adapterCameraSimple->activate();
			cout << "[" << PROGRAM_NAME << "]: CameraSimple adapter created in port " << tmp << endl;
		}
		catch (const IceStorm::TopicExists&){
			cout << "[" << PROGRAM_NAME << "]: ERROR creating or activating adapter for CameraSimple\n";
		}


		try
		{
			// Server adapter creation and publication
			if (not GenericMonitor::configGetString(communicator(), prefix, "DifferentialRobot.Endpoints", tmp, ""))
			{
				cout << "[" << PROGRAM_NAME << "]: Can't read configuration for proxy DifferentialRobot";
			}
			Ice::ObjectAdapterPtr adapterDifferentialRobot = communicator()->createObjectAdapterWithEndpoints("DifferentialRobot", tmp);
			auto differentialrobot = std::make_shared<DifferentialRobotI>(worker);
			adapterDifferentialRobot->add(differentialrobot, Ice::stringToIdentity("differentialrobot"));
			adapterDifferentialRobot->activate();
			cout << "[" << PROGRAM_NAME << "]: DifferentialRobot adapter created in port " << tmp << endl;
		}
		catch (const IceStorm::TopicExists&){
			cout << "[" << PROGRAM_NAME << "]: ERROR creating or activating adapter for DifferentialRobot\n";
		}


		try
		{
			// Server adapter creation and publication
			if (not GenericMonitor::configGetString(communicator(), prefix, "EmergencyStop.Endpoints", tmp, ""))
			{
				cout << "[" << PROGRAM_NAME << "]: Can't read configuration for proxy EmergencyStop";
			}
			Ice::ObjectAdapterPtr adapterEmergencyStop = communicator()->createObjectAdapterWithEndpoints("EmergencyStop", tmp);
			auto emergencystop = std::make_shared<EmergencyStopI>(worker);
			adapterEmergencyStop->add(emergencystop, Ice::stringToIdentity("emergencystop"));
			adapterEmergencyStop->activate();
			cout << "[" << PROGRAM_NAME << "]: EmergencyStop adapter created in port " << tmp << endl;
		}
		catch (const IceStorm::TopicExists&){
			cout << "[" << PROGRAM_NAME << "]: ERROR creating or activating adapter for EmergencyStop\n";
		}


		try
		{
			// Server adapter creation and publication
			if (not GenericMonitor::configGetString(communicator(), prefix, "EmotionalMotor.Endpoints", tmp, ""))
			{
				cout << "[" << PROGRAM_NAME << "]: Can't read configuration for proxy EmotionalMotor";
			}
			Ice::ObjectAdapterPtr adapterEmotionalMotor = communicator()->createObjectAdapterWithEndpoints("EmotionalMotor", tmp);
			auto emotionalmotor = std::make_shared<EmotionalMotorI>(worker);
			adapterEmotionalMotor->add(emotionalmotor, Ice::stringToIdentity("emotionalmotor"));
			adapterEmotionalMotor->activate();
			cout << "[" << PROGRAM_NAME << "]: EmotionalMotor adapter created in port " << tmp << endl;
		}
		catch (const IceStorm::TopicExists&){
			cout << "[" << PROGRAM_NAME << "]: ERROR creating or activating adapter for EmotionalMotor\n";
		}


		try
		{
			// Server adapter creation and publication
			if (not GenericMonitor::configGetString(communicator(), prefix, "LEDArray.Endpoints", tmp, ""))
			{
				cout << "[" << PROGRAM_NAME << "]: Can't read configuration for proxy LEDArray";
			}
			Ice::ObjectAdapterPtr adapterLEDArray = communicator()->createObjectAdapterWithEndpoints("LEDArray", tmp);
			auto ledarray = std::make_shared<LEDArrayI>(worker);
			adapterLEDArray->add(ledarray, Ice::stringToIdentity("ledarray"));
			adapterLEDArray->activate();
			cout << "[" << PROGRAM_NAME << "]: LEDArray adapter created in port " << tmp << endl;
		}
		catch (const IceStorm::TopicExists&){
			cout << "[" << PROGRAM_NAME << "]: ERROR creating or activating adapter for LEDArray\n";
		}


		try
		{
			// Server adapter creation and publication
			if (not GenericMonitor::configGetString(communicator(), prefix, "Laser.Endpoints", tmp, ""))
			{
				cout << "[" << PROGRAM_NAME << "]: Can't read configuration for proxy Laser";
			}
			Ice::ObjectAdapterPtr adapterLaser = communicator()->createObjectAdapterWithEndpoints("Laser", tmp);
			auto laser = std::make_shared<LaserI>(worker);
			adapterLaser->add(laser, Ice::stringToIdentity("laser"));
			adapterLaser->activate();
			cout << "[" << PROGRAM_NAME << "]: Laser adapter created in port " << tmp << endl;
		}
		catch (const IceStorm::TopicExists&){
			cout << "[" << PROGRAM_NAME << "]: ERROR creating or activating adapter for Laser\n";
		}


		try
		{
			// Server adapter creation and publication
			if (not GenericMonitor::configGetString(communicator(), prefix, "Speech.Endpoints", tmp, ""))
			{
				cout << "[" << PROGRAM_NAME << "]: Can't read configuration for proxy Speech";
			}
			Ice::ObjectAdapterPtr adapterSpeech = communicator()->createObjectAdapterWithEndpoints("Speech", tmp);
			auto speech = std::make_shared<SpeechI>(worker);
			adapterSpeech->add(speech, Ice::stringToIdentity("speech"));
			adapterSpeech->activate();
			cout << "[" << PROGRAM_NAME << "]: Speech adapter created in port " << tmp << endl;
		}
		catch (const IceStorm::TopicExists&){
			cout << "[" << PROGRAM_NAME << "]: ERROR creating or activating adapter for Speech\n";
		}


		// Server adapter creation and publication
		cout << SERVER_FULL_NAME " started" << endl;

		// User defined QtGui elements ( main window, dialogs, etc )

		#ifdef USE_QTGUI
			//ignoreInterrupt(); // Uncomment if you want the component to ignore console SIGINT signal (ctrl+c).
			a.setQuitOnLastWindowClosed( true );
		#endif
		// Run QT Application Event Loop
		a.exec();


		status = EXIT_SUCCESS;
	}
	catch(const Ice::Exception& ex)
	{
		status = EXIT_FAILURE;

		cout << "[" << PROGRAM_NAME << "]: Exception raised on main thread: " << endl;
		cout << ex;

	}
	#ifdef USE_QTGUI
		a.quit();
	#endif

	status = EXIT_SUCCESS;
	monitor->terminate();
	monitor->wait();
	delete worker;
	delete monitor;
	return status;
}

int main(int argc, char* argv[])
{
	string arg;

	// Set config file
	QString configFile("etc/config");
	bool startup_check_flag = false;
	QString prefix("");
	if (argc > 1)
	{

		// Search in argument list for arguments
		QString startup = QString("--startup-check");
		QString initIC = QString("--Ice.Config=");
		QString prfx = QString("--prefix=");
		for (int i = 0; i < argc; ++i)
		{
			arg = argv[i];
			if (arg.find(startup.toStdString(), 0) != std::string::npos)
			{
				startup_check_flag = true;
				cout << "Startup check = True"<< endl;
			}
			else if (arg.find(prfx.toStdString(), 0) != std::string::npos)
			{
				prefix = QString::fromStdString(arg).remove(0, prfx.size());
				if (prefix.size()>0)
					prefix += QString(".");
				printf("Configuration prefix: <%s>\n", prefix.toStdString().c_str());
			}
			else if (arg.find(initIC.toStdString(), 0) != std::string::npos)
			{
				configFile = QString::fromStdString(arg).remove(0, initIC.size());
				qDebug()<<__LINE__<<"Starting with config file:"<<configFile;
			}
			else if (i==1 and argc==2 and arg.find("--", 0) == std::string::npos)
			{
				configFile = QString::fromStdString(arg);
				qDebug()<<__LINE__<<QString::fromStdString(arg)<<argc<<arg.find("--", 0)<<"Starting with config file:"<<configFile;
			}
		}

	}
	::EboBridge app(prefix, startup_check_flag);

	return app.main(argc, argv, configFile.toLocal8Bit().data());
}
