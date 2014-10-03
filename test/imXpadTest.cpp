//###########################################################################
// This file is part of LImA, a Library for Image Acquisition
//
// Copyright (C) : 2009-2013
// European Synchrotron Radiation Facility
// BP 220, Grenoble 38043
// FRANCE
//
// This is free software; you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation; either version 3 of the License, or
// (at your option) any later version.
//
// This software is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program; if not, see <http://www.gnu.org/licenses/>.
//###########################################################################
#include "HwInterface.h"
#include "CtControl.h"
#include "CtAccumulation.h"
#include "CtAcquisition.h"
#include "CtSaving.h"
#include "CtShutter.h"
#include "Constants.h"

#include "imXpadCamera.h"
#include "imXpadInterface.h"
#include "Debug.h"
//#include "Exception.h"
#include <iostream>
#include <unistd.h>
#include <sys/stat.h>

using namespace std;
using namespace lima;
using namespace lima::imXpad;

DEB_GLOBAL(DebModTest);

int main(int argc, char *argv[])
{
    DEB_GLOBAL_FUNCT();
    //	DebParams::setModuleFlags(DebParams::AllFlags);
    //	DebParams::setTypeFlags(DebParams::AllFlags);
    DebParams::setTypeFlags(lima::DebTypeTrace);
    //	DebParams::setFormatFlags(DebParams::AllFlags);

    Camera *m_camera;
    Interface *m_interface;
    CtControl* m_control;

    //Xpad configuration properties
    string hostname = "127.0.0.1";
    string server_type = "PCI";
    string xpad_model = "XPAD_S1400";

    hostname = argv[1];
    int port = atoi(argv[2]);

    try {


        unsigned short *ret = new unsigned short[7*2];
        Camera::XpadStatus status;

        //MUST BE SET BEFORE CREATING AN INTERFACE
        m_camera = new Camera(hostname, port, xpad_model, server_type);
        m_camera->setImageType(Bpp16);
        m_camera->setLatTime(5000);
        m_camera->setTrigMode(lima::IntTrig);
        m_camera->setOutputSignalMode(Camera::XpadOutputSignal::ExposureBusy);
        m_camera->setGeometricalCorrectionsFlag(1);
        m_camera->setFlatFieldCorrectionsFlag(1);
        m_camera->setAcquisitionMode(Camera::XpadAcquisitionMode::BurstSSD);
        m_camera->setImageTransferFlag(0);
        m_camera->setImageFileFormat(Camera::XpadImageFileFormat::Binary);

        //CREATING AN INTERFACE AND A CONTROL
        m_interface = new Interface(*m_camera);
        m_control = new CtControl(m_interface);

        if(!m_camera->init()){

            //****DIGITAL TEST
            //m_camera->digitalTest(40,Camera::XpadDigitalTest::Strips);

            //****GLOBAL CONFIGURATION FONCTIONS
            //m_camera->loadConfigG(IMFP,5);
            //m_camera->loadConfigG(ITHL,32);
            //m_camera->readConfigG(ITUNE,ret);

            //cout << "Global Configuration readed with SUCCESS" << endl << "Register = "\
            //     << ret[0] << " Values: " << ret[1] << " " << ret[2] << " " << ret[3] << " "\
            //     << ret[4] << " " << ret[5] << " " << ret[6] << " " << ret[7];

            //m_camera->loadDefaultConfigGValues();
            //m_camera->ITHLIncrease();
            //m_camera->ITHLDecrease();


            //m_camera->saveConfigGToFile("./Calibration/ConfigGlobalTest.cfg");
            //m_camera->loadConfigGFromFile("./Calibration/ConfigGlobalTest.cfg");


            //****LOCAL CONFIGURATION FONCTIONS            
            //m_camera->saveConfigLToFile("./Calibration/ConfigLocalTest.cfl");
            //m_camera->loadConfigLFromFile("./Calibration/ConfigLocalTest.cfl");
            //m_camera->loadFlatConfigL(30);



            //****CALIBRATION OVER THE NOISE
            //mkdir("./Calibration",S_IRWXU |  S_IRWXG |  S_IRWXO);
            //m_camera->calibrationOTN(Camera::Calibration::Slow);
            //m_camera->calibrationOTNPulse(Camera::Calibration::Slow);
            //m_camera->calibrationBEAM(1000000, 50, Camera::Calibration::Slow);
            //m_camera->saveConfigLToFile("./Calibration/ConfigLocalSlow.cfl");
            //m_camera->saveConfigGToFile("./Calibration/ConfigGlobalSlow.cfg");



            CtSaving* saving = m_control->saving();
            saving->setDirectory("./Images");
            saving->setFormat(CtSaving::EDF);
            saving->setPrefix("id24_");
            saving->setSuffix(".edf");
            saving->setSavingMode(CtSaving::AutoFrame);
            //saving->setSavingMode(CtSaving::Manual);
            saving->setOverwritePolicy(CtSaving::Overwrite);

            int nframes = 1000;
            m_control->acquisition()->setAcqExpoTime(0.001);
            m_control->acquisition()->setAcqNbFrames(nframes);

            m_control->prepareAcq();
            m_control->startAcq();
            m_camera->stopAcq();

            //m_camera->resetModules();
            m_camera->exit();
        }

    } catch (Exception e) {
        DEB_ERROR() << "LIMA Exception: " << e;
    } catch (...) {
        DEB_ERROR() << "Unkown exception!";
    }
    return 0;
}
