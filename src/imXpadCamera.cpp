//###########################################################################
// This file is part of LImA, a Library for Image
//
// Copyright (C) : 2009-2011
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
//
// XpadCamera.cpp
// Created on: August 01, 2013
// Author: Hector PEREZ PONCE

#include <sstream>
#include <iostream>
#include <string>
#include <math.h>
#include <iomanip>
#include "imXpadCamera.h"
#include "Exceptions.h"
#include "Debug.h"
#include <unistd.h>
#include <sys/stat.h>
#include <ostream>
#include <fstream>


using namespace lima;
using namespace lima::imXpad;
using namespace std;


//---------------------------
//- utility thread
//---------------------------
class Camera::AcqThread: public Thread {
    DEB_CLASS_NAMESPC(DebModCamera, "Camera", "AcqThread");
public:
    AcqThread(Camera &aCam);
    virtual ~AcqThread();

protected:
    virtual void threadFunction();

private:
    Camera& m_cam;
};

//---------------------------
// @brief  Ctor
//---------------------------m_npixels

Camera::Camera(string hostname, int port, string xpad_model, string server_type) : m_hostname(hostname), m_port(port){
    DEB_CONSTRUCTOR();

    if (server_type == "PCI")
        m_server_type = PCI;
    else
        m_server_type = USB;


    /*
 * PCI MODE
#define HUB                         0
#define BACKPLANE                   1
#define XPAD_S70                    2
#define XPAD_S140                   3
#define XPAD_S340                   4
#define XPAD_S420                   5
#define XPAD_S540                   6
#define XPAD_S1400                  7

/*
 * USB MODE
#define XPAD_S10                    0
#define XPAD_C10                    1
#define XPAD_A10                    2
#define XPAD_S70                    3
#define XPAD_S70C                   4
#define XPAD_S140                   5
*/


    //	DebParams::setModuleFlags(DebParams::AllFlags);
    //  DebParams::setTypeFlags(DebParams::AllFlags);
    //	DebParams::setFormatFlags(DebParams::AllFlags);
    m_acq_thread = new AcqThread(*this);
    m_acq_thread->start();
    m_xpad = new XpadClient();
    
    if	(m_server_type == USB && xpad_model == "XPAD_S70"){
        m_xpad_model = 3; m_modules_mask = 1; m_chip_mask = 127; m_module_number = 1; m_chip_number = 7;
        m_image_size = Size(IMG_COLUMN * m_chip_number, IMG_LINE * m_module_number);
    }
    else if	(m_server_type == USB && xpad_model == "XPAD_S70C"){
        m_xpad_model = 4; m_modules_mask = 1; m_chip_mask = 127; m_module_number = 1; m_chip_number = 7;
        m_image_size = Size(IMG_COLUMN * m_chip_number, IMG_LINE * m_module_number);
    }
    else if	(m_server_type == PCI && xpad_model == "XPAD_S140"){
        m_xpad_model = 3; m_modules_mask = 3; m_chip_mask = 127; m_module_number = 2; m_chip_number = 7;
        m_image_size = Size(IMG_COLUMN * m_chip_number, IMG_LINE * m_module_number);
    }
    else if	(m_server_type == PCI && xpad_model == "XPAD_S1400"){
        m_xpad_model = 7; m_modules_mask = 1048575; m_chip_mask = 127; m_module_number = 20; m_chip_number = 7;
        m_image_size = Size(IMG_COLUMN * m_chip_number, IMG_LINE * m_module_number);
    }
    else
        THROW_HW_ERROR(Error) << "[ "<< "Xpad Model not supported"<< " ]";

    m_exp_time_usec = 1000000;
    m_lat_time_usec = 5000;
    m_xpad_trigger_mode = 0;
    m_xpad_output_signal_mode = 0;
    m_geometrical_corrections_flag = 1;
    m_flat_field_corrections_flag = 1;
    m_acquisition_mode = 0;
    m_image_transfer_flag = 1;
    m_image_file_format = 1;              //binary

    DEB_TRACE() << "--> Number of Modules 		 = " << m_module_number ;
    DEB_TRACE() << "--> Number of Chips 		 = " << m_chip_number ;
    DEB_TRACE() << "--> Image size               = " << m_image_size;

}

Camera::~Camera() {
    DEB_DESTRUCTOR();
    m_xpad->disconnectFromServer();
    delete m_xpad;
    delete m_acq_thread;
}

int Camera::init() {
    DEB_MEMBER_FUNCT();
    DEB_TRACE() << "********** Inside of Camera::init ***********";

    int ret;

    if (m_xpad->connectToServer(m_hostname, m_port) < 0) {
        THROW_HW_ERROR(Error) << "[ " << m_xpad->getErrorMessage() << " ]";
    }

    if(m_server_type == USB ){
        this->getUSBDeviceList();
        if (!this->setUSBDevice(0)){
            this->defineDetectorModel(m_xpad_model);
            ret = this->askReady();
        }
    }
    else{
        this->defineDetectorModel(m_xpad_model);
        ret = this->askReady();
    }

    DEB_TRACE() << "********** Outside of Camera::init ***********";

    return ret;
}

void Camera::reset() {
    DEB_MEMBER_FUNCT();
    DEB_TRACE() << "********** Inside of Camera::reset ***********";

    stringstream cmd1;
    cmd1 << "ResetModules";
    m_xpad->sendWait(cmd1.str());
    DEB_TRACE() << "Reset of detector  -> OK";

    DEB_TRACE() << "********** Outside of Camera::reset ***********";
}

void Camera::prepareAcq() {
    DEB_MEMBER_FUNCT();
    DEB_TRACE() << "********** Inside of Camera::prepareAcq ***********";

    int value;
    stringstream cmd1;

    m_image_file_format = 1;

    cmd1 << "SetExposeParameters " << m_nb_frames << " " << m_exp_time_usec << " "
         << 5000 << " " << m_xpad_trigger_mode << " " << m_xpad_output_signal_mode << " "
         << m_geometrical_corrections_flag << " " << m_flat_field_corrections_flag << " "
         << m_acquisition_mode << " " << m_image_transfer_flag << " " << m_image_file_format;

    m_xpad->sendWait(cmd1.str(), value);
    if(!value)
        DEB_TRACE() << "Default exposure parameter applied SUCCESFULLY";

    DEB_TRACE() << "********** Outside of Camera::prepareAcq ***********";
}

void Camera::startAcq() {
    DEB_MEMBER_FUNCT();
    DEB_TRACE() << "********** Inside of Camera::startAcq ***********";

    m_acq_frame_nb = 0;
    StdBufferCbMgr& buffer_mgr = m_bufferCtrlObj.getBuffer();
    buffer_mgr.setStartTimestamp(Timestamp::now());

    AutoMutex aLock(m_cond.mutex());
    m_wait_flag = false;
    m_quit = false;
    m_cond.broadcast();
    // Wait that Acq thread start if it's an external trigger
    while (m_xpad_trigger_mode == ExtTrigMult && !m_thread_running)
        m_cond.wait();

    DEB_TRACE() << "********** Outside of Camera::startAcq ***********";
}

void Camera::stopAcq() {
    DEB_MEMBER_FUNCT();
    DEB_TRACE() << "********** Inside of Camera::stopAcq ***********";

    AutoMutex aLock(m_cond.mutex());
    m_wait_flag = true;
    while (m_thread_running)
        m_cond.wait();

    DEB_TRACE() << "********** Outside of Camera::stopAcq ***********";
}


void Camera::sendExposeCommand(){
    DEB_MEMBER_FUNCT();
    DEB_TRACE() << "********** Inside of Camera::sendExposeCommmand ***********";

    m_xpad->sendExposeCommand();

    DEB_TRACE() << "********** Outside of Camera::sendExposeCommand ***********";
}

void Camera::readFrameExpose(void *bptr, int frame_nb) {
    DEB_MEMBER_FUNCT();
    DEB_TRACE() << "********** Inside of Camera::readFrameExpose ***********";

    DEB_TRACE() << "reading frame " << frame_nb;

    m_xpad->getDataExpose(bptr, m_xpad_format);

    DEB_TRACE() << "********** Outside of Camera::readFrameExpose ***********";
}

void Camera::readFrameExposeFromFile(void *bptr, int frame_nb) {
    DEB_MEMBER_FUNCT();
    DEB_TRACE() << "********** Inside of Camera::readFrameExposeFromFile ***********";

    DEB_TRACE() << "reading frame " << frame_nb;

    m_xpad->getDataExposeFromFile(bptr, m_xpad_format);

    DEB_TRACE() << "********** Outside of Camera::readFrameExposeFromFile ***********";
}

int Camera::getDataExposeReturn() {
    DEB_MEMBER_FUNCT();
    DEB_TRACE() << "********** Inside of Camera::getExposeCommandReturn ***********";

    int ret;
    m_xpad->getExposeCommandReturn(ret);

    DEB_TRACE() << "********** Outside of Camera::getExposeCommandReturn ***********";

    return ret;
}

void Camera::getStatus(XpadStatus& status,bool internal) {
    DEB_MEMBER_FUNCT();
    DEB_TRACE() << "********** Inside of Camera::getStatus ***********";

    stringstream cmd;
    string str;
    unsigned short pos;
    cmd << "GetStatus";
    
    if(internal || !m_thread_running)
    {
        m_xpad->sendWait(cmd.str(), str);
        pos = str.find(":");
        string state = str.substr (0, pos);
        if (state.compare("Idle") == 0) {
            status.state = XpadStatus::Idle;
        } else if (state.compare("Digital_Test") == 0) {
            status.state = XpadStatus::Test;
        } else if (state.compare("Resetting") == 0) {
            status.state = XpadStatus::Resetting;
        } else {
            status.state = XpadStatus::Running;
        }
    }
    else
        status.state = XpadStatus::Running;

    DEB_TRACE() << "XpadStatus.state is [" << status.state << "]";

    DEB_TRACE() << "********** Outside of Camera::getStatus ***********";

}

int Camera::getNbHwAcquiredFrames() {
    DEB_MEMBER_FUNCT();
    return m_acq_frame_nb;
}

void Camera::AcqThread::threadFunction() {
    DEB_MEMBER_FUNCT();
    AutoMutex aLock(m_cam.m_cond.mutex());
    StdBufferCbMgr& buffer_mgr = m_cam.m_bufferCtrlObj.getBuffer();

    while (!m_cam.m_quit) {
        while (m_cam.m_wait_flag && !m_cam.m_quit) {
            DEB_TRACE() << "Wait";
            m_cam.m_thread_running = false;
            m_cam.m_cond.broadcast();
            m_cam.m_cond.wait();
        }
        DEB_TRACE() << "AcqThread Running";
        m_cam.m_thread_running = true;
        if (m_cam.m_quit)
            return;

        m_cam.m_cond.broadcast();
        aLock.unlock();

        XpadStatus status;
        m_cam.getStatus(status,true);
        if (status.state == status.Idle){
            m_cam.sendExposeCommand();

            bool continueFlag = true;
            if (m_cam.m_image_transfer_flag == 1){
                while (continueFlag && (!m_cam.m_nb_frames || m_cam.m_acq_frame_nb < m_cam.m_nb_frames)) {

                    DEB_TRACE() << m_cam.m_acq_frame_nb;
                    void *bptr = buffer_mgr.getFrameBufferPtr(m_cam.m_acq_frame_nb);

                    m_cam.readFrameExpose(bptr, m_cam.m_acq_frame_nb);

                    HwFrameInfoType frame_info;
                    frame_info.acq_frame_nb = m_cam.m_acq_frame_nb;
                    continueFlag = buffer_mgr.newFrameReady(frame_info);
                    DEB_TRACE() << "acqThread::threadFunction() newframe ready ";
                    ++m_cam.m_acq_frame_nb;

                    DEB_TRACE() << "acquired " << m_cam.m_acq_frame_nb << " frames, required " << m_cam.m_nb_frames << " frames";
                }
            }
            else{
                while (continueFlag && (!m_cam.m_nb_frames || m_cam.m_acq_frame_nb < m_cam.m_nb_frames)) {


                    void *bptr = buffer_mgr.getFrameBufferPtr(m_cam.m_acq_frame_nb);

                    //m_cam.readFrameExposeFromFile(bptr, m_cam.m_acq_frame_nb);

                    uint numData = m_cam.m_image_size.getWidth() * m_cam.m_image_size.getHeight();

                    DEB_TRACE() << m_cam.m_acq_frame_nb;
                    DEB_TRACE() << m_cam.m_image_size.getWidth() << " " << m_cam.m_image_size.getHeight();

                    uint16_t *buffer_short;
                    uint32_t *buffer_int;


                    if (m_cam.m_xpad_format == 0)
                        buffer_short = (uint16_t *)bptr;
                    else
                        buffer_int = (uint32_t *)bptr;

					if(m_cam.m_acq_frame_nb <  m_cam.m_nb_frames - 1){
						stringstream fileName1;
						fileName1 << "/opt/imXPAD/tmp_corrected/file_" << m_cam.m_acq_frame_nb + 1 << ".bin";
						while (access( fileName1.str().c_str(), F_OK ) == -1);
					}
					else
						usleep(60000);
                    
                    stringstream fileName2;
                    fileName2 << "/opt/imXPAD/tmp_corrected/file_" << m_cam.m_acq_frame_nb  << ".bin";

                    ifstream file(fileName2.str().c_str(), ios::in|ios::binary);

                    unsigned char data[numData*sizeof(int32_t)];
                    int32_t *dataBuff = new int32_t[numData];

                    if (file.is_open()){

                        DEB_TRACE() << "FILE OPEN";
                        file.read((char *)&data, numData*sizeof(int32_t));

                        uint32_t count = 0;
                        int i=0;
                        while (count < numData*sizeof(int32_t)){

                            if (m_cam.m_xpad_format==0){
                                memcpy (&dataBuff[i], &data[count], sizeof(int32_t) );
                                buffer_short[i] = (uint16_t)(dataBuff[i]);
                                    //cout << dataBuff[i] << " ";

                            }
                            else{
                                memcpy (&dataBuff[i], &data[count], sizeof(int32_t) );
                                buffer_int[i] = (uint32_t)(dataBuff[i]);
                                    //cout << dataBuff[i] << " ";
                            }
                            count += sizeof(int32_t);
                            i++;
                        }
                    }

                    delete[] dataBuff;

                    file.close();

                    HwFrameInfoType frame_info;
                    frame_info.acq_frame_nb = m_cam.m_acq_frame_nb;
                    continueFlag = buffer_mgr.newFrameReady(frame_info);
                    DEB_TRACE() << "acqThread::threadFunction() newframe ready ";
                    ++m_cam.m_acq_frame_nb;

                    DEB_TRACE() << "acquired " << m_cam.m_acq_frame_nb << " frames, required " << m_cam.m_nb_frames << " frames";
                }
            }

            if (m_cam.getDataExposeReturn() == 0)
                DEB_TRACE() << "Images received SUCCESFULLY from server";
            else
                throw LIMA_HW_EXC(Error, "Receiving images from server FAILED");
        }
        else {
            AutoMutex aLock(m_cam.m_cond.mutex());
            usleep(1000);
        }
        aLock.lock();
        m_cam.m_wait_flag = true;
    }
}

Camera::AcqThread::AcqThread(Camera& cam) :
    m_cam(cam) {
    AutoMutex aLock(m_cam.m_cond.mutex());
    m_cam.m_wait_flag = true;
    m_cam.m_quit = false;
    aLock.unlock();
    pthread_attr_setscope(&m_thread_attr, PTHREAD_SCOPE_PROCESS);
}

Camera::AcqThread::~AcqThread() {
    AutoMutex aLock(m_cam.m_cond.mutex());
    m_cam.m_quit = true;
    m_cam.m_cond.broadcast();
    aLock.unlock();
}

void Camera::getDetectorImageSize(Size& size) {
    DEB_MEMBER_FUNCT();
    //size = Size(m_npixels, 1);
    size = m_image_size;
}

void Camera::getPixelSize(double& size_x, double& size_y) {
    DEB_MEMBER_FUNCT();
    //sizex = xPixelSize;
    //sizey = yPixelSize;
    size_x = 130; // pixel size is 130 micron
    size_y = 130;
}

void Camera::getImageType(ImageType& pixel_depth) {
    DEB_MEMBER_FUNCT();
    //type = m_image_type;
    switch( m_xpad_format )
    {
    case 0:
        pixel_depth = Bpp16;
        break;

    case 1:
        pixel_depth = Bpp32;
        break;
    }
}

void Camera::setImageType(ImageType pixel_depth) {
    //this is what xpad generates but may need Bpp32 for accummulate
    DEB_MEMBER_FUNCT();
    //m_image_type = type;
    switch( pixel_depth )
    {
    case Bpp16:
        m_pixel_depth = B2;
        m_xpad_format = 0;
        break;

    case Bpp32:
        m_pixel_depth = B4;
        m_xpad_format = 1;
        break;
    default:
        DEB_ERROR() << "Pixel Depth is unsupported: only 16 or 32 bits is supported" ;
        throw LIMA_HW_EXC(Error, "Pixel Depth is unsupported: only 16 or 32 bits is supported");
        break;
    }
}

void Camera::getDetectorType(std::string& type) {
    DEB_MEMBER_FUNCT();
    type = "XPAD";
}

void Camera::getDetectorModel(std::string& model) {
    DEB_MEMBER_FUNCT();
    if (m_server_type == PCI){
        if(m_xpad_model == 0) model = "HUB";
        else if(m_xpad_model == 1) model = "BACKPLANE";
        else if(m_xpad_model == 2) model = "XPAD_S70";
        else if(m_xpad_model == 3) model = "XPAD_S140";
        else if(m_xpad_model == 4) model = "XPAD_S340";
        else if(m_xpad_model == 5) model = "XPAD_S420";
        else if(m_xpad_model == 6) model = "XPAD_S540";
        else if(m_xpad_model == 7) model = "XPAD_S1400";
        else throw LIMA_HW_EXC(Error, "Xpad Type not supported");
    }
    else{
        if(m_xpad_model == 0) model = "XPAD_S10";
        else if(m_xpad_model == 1) model = "XPAD_C10";
        else if(m_xpad_model == 2) model = "XPAD_A10";
        else if(m_xpad_model == 3) model = "XPAD_S70";
        else if(m_xpad_model == 4) model = "XPAD_S70C";
        else if(m_xpad_model == 5) model = "XPAD_S140";
        else throw LIMA_HW_EXC(Error, "Xpad Type not supported");
    }
}

HwBufferCtrlObj* Camera::getBufferCtrlObj() {
    return &m_bufferCtrlObj;
}


void Camera::setTrigMode(TrigMode mode) {
    DEB_MEMBER_FUNCT();
    DEB_TRACE() << "Camera::setTrigMode - " << DEB_VAR1(mode);
    DEB_PARAM() << DEB_VAR1(mode);

    switch( mode )
    {
    case IntTrig:
        m_xpad_trigger_mode = 0;
        break;
    case ExtGate:
        m_xpad_trigger_mode = 1;
        break;
    case ExtTrigSingle:
        m_xpad_trigger_mode = 2;
        break;
    case ExtTrigMult:
        m_xpad_trigger_mode = 3;
        break;
    default:
        DEB_ERROR() << "Error: Trigger mode unsupported: only IntTrig, ExtGate or ExtTrigSingle" ;
        throw LIMA_HW_EXC(Error, "Trigger mode unsupported: only IntTrig, ExtGate or ExtTrigSingle");
        break;
    }
}

void Camera::getTrigMode(TrigMode& mode) {
    DEB_MEMBER_FUNCT();
    //mode = m_trigger_mode;
    switch( m_xpad_trigger_mode )
    {
    case 0:
        mode = IntTrig;
        break;
    case 1:
        mode = ExtGate;
        break;
    case 2:
        mode = ExtTrigSingle;
        break;
    case 3:
        mode = ExtTrigMult;
        break;
    default:
        break;
    }
    DEB_RETURN() << DEB_VAR1(mode);
}


void Camera::setExpTime(double exp_time_sec) {
    DEB_MEMBER_FUNCT();
    DEB_TRACE() << "Camera::setExpTime - " << DEB_VAR1(exp_time_sec);

    m_exp_time_usec = exp_time_sec * 1e6;
}

void Camera::getExpTime(double& exp_time_sec) {
    DEB_MEMBER_FUNCT();
    DEB_TRACE() << "Camera::getExpTime";
    ////	AutoMutex aLock(m_cond.mutex());

    exp_time_sec = m_exp_time_usec / 1e6;
    DEB_RETURN() << DEB_VAR1(exp_time_sec);
}

void Camera::setLatTime(double lat_time) {
    DEB_MEMBER_FUNCT();
    DEB_PARAM() << "Camera::setLatTime - " << DEB_VAR1(lat_time);

    m_lat_time_usec = lat_time;
}

void Camera::getLatTime(double& lat_time) {
    DEB_MEMBER_FUNCT();
    DEB_TRACE() << "Camera::getLatTime";

    lat_time = m_lat_time_usec;
    DEB_RETURN() << DEB_VAR1(lat_time);
}

void Camera::setNbFrames(int nb_frames) {
    DEB_MEMBER_FUNCT();
    DEB_TRACE() << "Camera::setNbFrames - " << DEB_VAR1(nb_frames);
    if (m_nb_frames < 0) {
        THROW_HW_ERROR(Error) << "Number of frames to acquire has not been set";
    }
    m_nb_frames = nb_frames;
}

void Camera::getNbFrames(int& nb_frames) {
    DEB_MEMBER_FUNCT();
    DEB_TRACE() << "Camera::getNbFrames";
    DEB_RETURN() << DEB_VAR1(m_nb_frames);
    nb_frames = m_nb_frames;
}

bool Camera::isAcqRunning() const {
    AutoMutex aLock(m_cond.mutex());
    return m_thread_running;
}

/////////////////////////
// XPAD Specific
/////////////////////////

string Camera::getUSBDeviceList(void){
    DEB_MEMBER_FUNCT();
    DEB_TRACE() << "********** Inside of Camera::getUSBDeviceList ***********";

    string message;
    stringstream cmd;

    cmd.str(string());
    cmd << "GetUSBDeviceList";
    m_xpad->sendWait(cmd.str(), message);

    DEB_TRACE() << "List of USB devices connected: " << message;

    DEB_TRACE() << "********** Outside of Camera::getUSBDeviceList ***********";

    return message;

}

int Camera::setUSBDevice(unsigned short device){
    DEB_MEMBER_FUNCT();
    DEB_TRACE() << "********** Inside of Camera::setUSBDevice ***********";

    int ret;
    stringstream cmd;

    cmd.str(string());
    cmd << "SetUSBDevice " << device;
    m_xpad->sendWait(cmd.str(), ret);

    if(!ret)
        DEB_TRACE() << "Setting active USB device to " << device;
    else
        throw LIMA_HW_EXC(Error, "Setting USB device FAILED!");

    DEB_TRACE() << "********** Outside of Camera::setUSBDevice ***********";

    return ret;

}

int Camera::defineDetectorModel(unsigned short model){
    DEB_MEMBER_FUNCT();
    DEB_TRACE() << "********** Inside of Camera::DefineDetectorModel ***********";

    int ret;
    stringstream cmd;

    cmd.str(string());
    cmd << "DefineDetectorModel " << model;
    m_xpad->sendWait(cmd.str(), ret);

    if(!ret)
        DEB_TRACE() << "Defining detector model to " << model;
    else
        throw LIMA_HW_EXC(Error, "Defining detector model FAILED");


    DEB_TRACE() << "********** Outside of Camera::DefineDetectorModel ***********";

    return ret;

}

int Camera::askReady(){
    DEB_MEMBER_FUNCT();
    DEB_TRACE() << "********** Inside of Camera::askReady ***********";

    int ret;
    stringstream cmd;

    cmd.str(string());
    cmd << "AskReady";
    m_xpad->sendWait(cmd.str(), ret);

    if(!ret)
        DEB_TRACE() << "Module " << m_modules_mask << " is READY";
    else
        throw LIMA_HW_EXC(Error, "AskReady FAILED!");

    DEB_TRACE() << "********** Outside of Camera::askRead ***********";

    return ret;
}

int Camera::digitalTest(unsigned short DT_value, unsigned short mode){
    DEB_MEMBER_FUNCT();
    DEB_TRACE() << "********** Inside of Camera::digitalTest ***********";

    int ret;
    stringstream cmd;

    cmd.str(string());
    cmd << "DigitalTest " << DT_value << " " << mode ;
    m_xpad->sendNoWait(cmd.str());

    int rows = IMG_LINE * m_module_number;
    int columns = IMG_COLUMN * m_chip_number;

    int32_t buff[rows * columns];
    this->readFrameExpose(buff, 1);

    int32_t val;
    //Saving Digital Test image to disk
    mkdir("./Images",S_IRWXU |  S_IRWXG |  S_IRWXO);
    ofstream file("./Images/DigitalTest.bin", ios::out|ios::binary);
    if (file.is_open()){
        for (int i=0;i<rows;i++) {
            for (int j=0;j<columns;j++){
                val = buff[i*columns+j];
                file.write((char *)&val, sizeof(int32_t));
            }
        }
    }
    file.close();

    ret = this->getDataExposeReturn();

    if(!ret)
        DEB_TRACE() << "Digital Test performed SUCCESFULLY";
    else
        throw LIMA_HW_EXC(Error, "Digital Test FAILED!");

    DEB_TRACE() << "********** Outside of Camera::digitalTest ***********";

    return ret;
}

int Camera::loadConfigGFromFile(char *fpath){
    DEB_MEMBER_FUNCT();
    DEB_TRACE() << "********** Inside of Camera::loadConfigGFromFile ***********";

    int ret;
    stringstream cmd;

    ifstream file(fpath, ios::in);

    //Opening the file to read
    if (file.is_open()){

        string str;
        stringstream data;
        int dataSize;

        cmd.str(string());
        cmd << "LoadConfigGFromFile ";

        m_xpad->sendWait(cmd.str(), str);
        if (str.compare("SERVER: Ready to receive data")==0){
            cmd.str(string());
            while(!file.eof()){
                char temp;
                file.read(&temp,sizeof(char));
                data << temp;
            }
            string temp = data.str();
            cmd << temp.size();
            m_xpad->sendWait(cmd.str(), dataSize);

            if(temp.size() == dataSize){
                m_xpad->sendWait("OK", temp);
                m_xpad->sendWait(data.str(), temp);
                m_xpad->sendWait("Waiting for answer",ret);
            }
            else{
                m_xpad->sendWait("ERROR", ret);
            }
        }
        file.close();
    }

    if(!ret)
        DEB_TRACE() << "Global configuration loaded from file SUCCESFULLY";
    else
        throw LIMA_HW_EXC(Error, "Loading global configuration from file FAILED!");

    DEB_TRACE() << "********** Outside of Camera::loadConfigGFromFile ***********";

    return ret;
}

int Camera::saveConfigGToFile(char *fpath){
    DEB_MEMBER_FUNCT();
    DEB_TRACE() << "********** Inside of Camera::saveConfigGToFile ***********";

    int ret;
    unsigned short  regid;

    stringstream cmd;

    //File is being open to be writen
    ofstream file(fpath, ios::out);
    if (file.is_open()){

        string          retString;

        //All registers are being read
        for (unsigned short registro=0; registro<7; registro++){
            switch(registro){
            case 0: regid=AMPTP;break;
            case 1: regid=IMFP;break;
            case 2: regid=IOTA;break;
            case 3: regid=IPRE;break;
            case 4: regid=ITHL;break;
            case 5: regid=ITUNE;break;
            case 6: regid=IBUFF;
            }

            //Each register value is read from the detector
            cmd.str(string());
            cmd << "ReadConfigG " << regid;
            m_xpad->sendWait(cmd.str(), retString);

            stringstream stream(retString.c_str());
            int length = retString.length();

            int mdMask = 1;
            while (mdMask<m_modules_mask){
                if (length > 1){
                    //Register values are being stored in the file
                    file << mdMask << " ";
                    for(int count=0; count<8; count++){
                        stream >> retString;
                        file << retString << " ";
                    }
                    file << endl;
                    mdMask = mdMask << 1;
                    ret = 0;
                }
                else{
                    ret = -1;
                }
            }
        }
        file.close();
    }

    if(!ret)
        DEB_TRACE() << "Global configuration saved to file SUCCESFULLY";
    else
        throw LIMA_HW_EXC(Error, "Saving global configuration to file FAILED!");


    DEB_TRACE() << "********** Outside of Camera::saveConfigGToFile ***********";

    return ret;
}

int Camera::loadConfigG(unsigned short reg, unsigned short value){
    DEB_MEMBER_FUNCT();
    DEB_TRACE() << "********** Inside of Camera::loadConfigG ***********";

    string ret;
    stringstream cmd;

    cmd.str(string());
    cmd << "LoadConfigG "<< reg << " " << value ;
    m_xpad->sendWait(cmd.str(), ret);

    DEB_TRACE() << "********** Outside of Camera::loadConfigG ***********";

    if(ret.length()>1){
        DEB_TRACE() << "Loading global register: " << reg << " with value= " << value;
        return 0;
    }
    else{
        throw LIMA_HW_EXC(Error, "Loading global configuration FAILED!");
        return -1;
    }


}

int Camera::readConfigG(unsigned short reg, void *values){
    DEB_MEMBER_FUNCT();
    DEB_TRACE() << "********** Inside of Camera::readConfigG ***********";

    string message;
    stringstream cmd;

    unsigned short *ret = (unsigned short *)values;

    cmd.str(string());
    cmd << "ReadConfigG " << " " << reg;
    m_xpad->sendWait(cmd.str(), message);

    if(message.length()>1){
        char *value = new char[message.length()+1];
        strcpy(value,message.c_str());

        char *p = strtok(value," ,.");
        ret[0] = (unsigned short)atoi(p);
        for (int i=1; i<8 ; i++){
            p = strtok(NULL, " ,.");
            ret[i] = (unsigned short)atoi(p);
        }
    }
    else
        ret = NULL;

    DEB_TRACE() << "********** Outside of Camera::readConfigG ***********";

    if(ret!=NULL){
        DEB_TRACE() << "Reading global configuration performed SUCCESFULLY";
        return 0;
    }
    else{
        throw LIMA_HW_EXC(Error, "Reading global configuration FAILED!");
        return -1;
    }


}

int Camera::loadDefaultConfigGValues(){
    DEB_MEMBER_FUNCT();
    DEB_TRACE() << "********** Inside of Camera::LoadDefaultConfigGValues ***********";

    string ret;
    stringstream cmd;
    int value, regid;

    for (int i=0; i<7; i++){
        switch (i){
        case 0: regid = AMPTP;value = 0;break;
        case 1: regid = IMFP;value = 50;break;
        case 2: regid = IOTA;value = 40;break;
        case 3: regid = IPRE;value = 60;break;
        case 4: regid = ITHL;value = 25;break;
        case 5: regid = ITUNE;value = 100;break;
        case 6: regid = IBUFF;value = 0;
        }
        cmd.str(string());
        cmd << "LoadConfigG "<< regid << " " << value ;
        m_xpad->sendWait(cmd.str(), ret);
    }

    DEB_TRACE() << "********** Outside of Camera::LoadDefaultConfigGValues ***********";

    if(ret.length()>1){
        DEB_TRACE() << "Loading global configuratioin with values:\nAMPTP = 0, IMFP = 50, IOTA = 40, IPRE = 60, ITHL = 25, ITUNE = 100, IBUFF = 0";
        return 0;
    }
    else{
        throw LIMA_HW_EXC(Error, "Loading default global configuration values FAILED!");
        return -1;
    }

}

int Camera::ITHLIncrease(){
    DEB_MEMBER_FUNCT();
    DEB_TRACE() << "********** Inside of Camera::ITHLIncrease ***********";

    int ret;
    stringstream cmd;

    cmd.str(string());
    cmd << "ITHLIncrease";
    m_xpad->sendWait(cmd.str(), ret);

    if(!ret)
        DEB_TRACE() << "ITHL was increased SUCCESFULLY";
    else
        throw LIMA_HW_EXC(Error, "ITHL increase FAILED!");

    DEB_TRACE() << "********** Outside of Camera::ITHLIncrease ***********";

    return ret;
}

int Camera::ITHLDecrease(){
    DEB_MEMBER_FUNCT();
    DEB_TRACE() << "********** Inside of Camera::ITHLDecrease ***********";

    int ret;
    stringstream cmd;

    cmd.str(string());
    cmd << "ITHLDecrease";
    m_xpad->sendWait(cmd.str(), ret);

    if(!ret)
        DEB_TRACE() << "ITHL was decreased SUCCESFULLY";
    else
        throw LIMA_HW_EXC(Error, "ITHL decrease FAILED!");

    DEB_TRACE() << "********** Outside of Camera::ITHLDecrease ***********";

    return ret;
}

int Camera::loadFlatConfigL(unsigned short flat_value)
{
    DEB_MEMBER_FUNCT();

    DEB_TRACE() << "********** Inside of Camera::loadFlatConfig ***********";

    int ret;
    stringstream cmd;

    cmd.str(string());
    cmd <<  "LoadFlatConfigL " << " " << flat_value*8+1;
    m_xpad->sendWait(cmd.str(), ret);

    if(!ret)
        DEB_TRACE() << "Loading local configurations values, with flat value: " <<  flat_value << " -> OK" ;
    else
        throw LIMA_HW_EXC(Error, "Loading local configuratin with FLAT value FAILED!");

    DEB_TRACE() << "********** Outside of Camera::loadFlatConfig ***********";

    return ret;
}

int Camera::loadConfigLFromFile(char *fpath){
    DEB_MEMBER_FUNCT();
    DEB_TRACE() << "********** Inside of Camera::loadConfigLFromFile ***********";

    int ret;
    stringstream cmd;

    ifstream file(fpath, ios::in);

    //Opening the file to read
    if (file.is_open()){

        string str;
        stringstream data;
        int dataSize;

        cmd.str(string());
        cmd << "LoadConfigLFromFile ";

        m_xpad->sendWait(cmd.str(), str);
        if (str.compare("SERVER: Ready to receive data")==0){
            cmd.str(string());
            while(!file.eof()){
                char temp;
                file.read(&temp,sizeof(char));
                data << temp;
            }
            string temp = data.str();
            cmd << temp.size();
            m_xpad->sendWait(cmd.str(), dataSize);

            if(temp.size() == dataSize){
                m_xpad->sendWait("OK", str);
                if(str.compare("SERVER: OK")==0){
                    m_xpad->sendWait(data.str(), str);
                    m_xpad->sendWait("Waiting for answer",ret);
                }
            }
            else{
                m_xpad->sendWait("ERROR", ret);
            }
        }
        file.close();
    }

    if(!ret)
        DEB_TRACE() << "Local configuration loaded from file SUCCESFULLY";
    else
        throw LIMA_HW_EXC(Error, "Loading local configuration from file FAILED!");

    DEB_TRACE() << "********** Outside of Camera::loadConfigLFromFile ***********";

    return ret;


}

int Camera::saveConfigLToFile(char *fpath){
    DEB_MEMBER_FUNCT();
    DEB_TRACE() << "********** Inside of Camera::saveConfigLToFile ***********";

    stringstream cmd;
    string str;
    int ret, dataSize;

    cmd << "ReadConfigL";
    m_xpad->sendWait(cmd.str(),str);
    if (str.compare("SERVER: Ready to send data")==0){
        cmd.str(std::string());

        cmd << "CLIENT: Ready to receive dataSize";
        m_xpad->sendWait(cmd.str(), dataSize);

        DEB_TRACE() << "Receiving: "  << dataSize;

        cmd.str(std::string());
        cmd << dataSize;
        m_xpad->sendWait(cmd.str(), ret);

        if(ret == 0){
            m_xpad->sendNoWait("OK");

            ofstream file(fpath, ios::out);
            if (file.is_open()){
                for(int i=0; i<dataSize; i++)
                    file << (char) m_xpad->getChar();
                file.close();
                //cout << "Total of data sent: " << dataCount << endl;
            }
        }
    }
    else
        ret = -1;

    if(ret == 0)
        DEB_TRACE() << "Local configuration was saved to file SUCCESFULLY";
    else
        throw LIMA_HW_EXC(Error, "Saving local configuration to file FAILED!");

    DEB_TRACE() << "********** Outside of Camera::saveConfigLToFile ***********";

    return ret;
}

void Camera::setOutputSignalMode(unsigned short mode) {
    DEB_MEMBER_FUNCT();
    DEB_TRACE() << "Camera::setTrigMode - " << DEB_VAR1(mode);
    DEB_PARAM() << DEB_VAR1(mode);

    if (mode >=0 && mode<9 )
        m_xpad_output_signal_mode = mode;
    else{
        DEB_ERROR() << "Error: Output Signal mode unsupported" ;
        throw LIMA_HW_EXC(Error, "Output Signal mode unsupported");
    }
}

void Camera::getOutputSignalMode(unsigned short& mode) {
    DEB_MEMBER_FUNCT();
    DEB_TRACE() << "Camera::getOutputSignalMode - " << DEB_VAR1(mode);
    DEB_PARAM() << DEB_VAR1(mode);

    mode = m_xpad_output_signal_mode;
}

void Camera:: setGeometricalCorrectionsFlag(unsigned short flag){
    DEB_MEMBER_FUNCT();
    DEB_TRACE() << "Camera::setGeometricalCorretctionsFlag - " << DEB_VAR1(flag);
    DEB_PARAM() << DEB_VAR1(flag);

    m_geometrical_corrections_flag = flag;

    if	(m_server_type == USB && m_xpad_model == 3){
        if (m_geometrical_corrections_flag)
            m_image_size = Size(576 , 118);
        else
            m_image_size = Size(IMG_COLUMN * m_chip_number, IMG_LINE * m_module_number);
    }
    else if	(m_server_type == USB && m_xpad_model == 4){
        if (m_geometrical_corrections_flag)
            m_image_size = Size(576 , 118);
        else
            m_image_size = Size(IMG_COLUMN * m_chip_number, IMG_LINE * m_module_number);
    }
    else if	(m_server_type == PCI && m_xpad_model == 3){
        if (m_geometrical_corrections_flag)
            m_image_size = Size(576 , 236);
        else
            m_image_size = Size(IMG_COLUMN * m_chip_number, IMG_LINE * m_module_number);
    }
    else if	(m_server_type == PCI && m_xpad_model == 7){
        if (m_geometrical_corrections_flag)
            m_image_size = Size(1168 , 1116);
        else
            m_image_size = Size(IMG_COLUMN * m_chip_number, IMG_LINE * m_module_number);
    }
}

void Camera::setFlatFieldCorrectionsFlag(unsigned short flag){
    DEB_MEMBER_FUNCT();
    DEB_TRACE() << "Camera::setFlatFieldFlag - " << DEB_VAR1(flag);
    DEB_PARAM() << DEB_VAR1(flag);

    m_flat_field_corrections_flag = flag;
}

void Camera::setAcquisitionMode(unsigned int mode){
    DEB_MEMBER_FUNCT();
    DEB_TRACE() << "Camera::setAcquisitionMode - " << DEB_VAR1(mode);
    DEB_PARAM() << DEB_VAR1(mode);

    m_acquisition_mode = mode;
}

void Camera::setImageTransferFlag(unsigned short flag){
    DEB_MEMBER_FUNCT();
    DEB_TRACE() << "Camera::setTransferFlag - " << DEB_VAR1(flag);
    DEB_PARAM() << DEB_VAR1(flag);

    m_image_transfer_flag = flag;
}

void Camera::setImageFileFormat(unsigned short format){
    DEB_MEMBER_FUNCT();
    DEB_TRACE() << "Camera::ssetImageFileFormat - " << DEB_VAR1(format);
    DEB_PARAM() << DEB_VAR1(format);

    m_image_file_format = format;
}

int Camera::calibrationOTN(unsigned short calibrationConfiguration){

    DEB_MEMBER_FUNCT();
    DEB_TRACE() << "********** Inside of Camera::calibrationOTN ***********";

    int ret;
    stringstream cmd;

    cmd <<  "CalibrationOTN " << calibrationConfiguration;
    m_xpad->sendWait(cmd.str(), ret);

    if(!ret)
        DEB_TRACE() << "Calibration Over The Noise was SUCCESFULL ";
    else
        throw LIMA_HW_EXC(Error, "Calibration Over The Noise FAILED!");

    DEB_TRACE() << "********** Outside of Camera::calibrationOTN ***********";

    return ret;

}

int Camera::calibrationOTNPulse(unsigned short calibrationConfiguration){

    DEB_MEMBER_FUNCT();
    DEB_TRACE() << "********** Inside of Camera::calibrationOTNPulse ***********";

    int ret;
    stringstream cmd;

    cmd <<  "CalibrationOTNPulse " << calibrationConfiguration;
    m_xpad->sendWait(cmd.str(), ret);

    if(!ret)
        DEB_TRACE() << "Calibration Over The Noise with PULSE was SUCCESFULL ";
    else
        throw LIMA_HW_EXC(Error, "Calibration Over The Noise with PULSE FAILED!");

    DEB_TRACE() << "********** Outside of Camera::calibrationOTNPulse ***********";

    return ret;

}

int Camera::calibrationBEAM(unsigned int time, unsigned int ITHLmax, unsigned short calibrationConfiguration){

    DEB_MEMBER_FUNCT();
    DEB_TRACE() << "********** Inside of Camera::calibrationBEAM ***********";

    int ret;
    stringstream cmd;

    cmd <<  "CalibrationOTNPulse " << time << " " << ITHLmax << " " << calibrationConfiguration;
    m_xpad->sendWait(cmd.str(), ret);

    if(!ret)
        DEB_TRACE() << "Calibration BEAM was SUCCESFULL ";
    else
        throw LIMA_HW_EXC(Error, "Calibration BEAM FAILED!");

    DEB_TRACE() << "********** Outside of Camera::calibrationBEAM ***********";

    return ret;

}

int Camera::resetModules(){

    DEB_MEMBER_FUNCT();
    DEB_TRACE() << "********** Inside of Camera::calibrationOTN ***********";

    int ret;
    stringstream cmd;

    cmd <<  "ResetModules ";
    m_xpad->sendWait(cmd.str(), ret);

    if(!ret)
        DEB_TRACE() << "Modules were reset SUCCESFULLY ";
    else
        throw LIMA_HW_EXC(Error, "Resetting modules FAILED!");

    DEB_TRACE() << "********** Outside of Camera::calibrationOTN ***********";

    return ret;

}

void Camera::exit(){

    DEB_MEMBER_FUNCT();
    DEB_TRACE() << "********** Inside of Camera::exit ***********";
    stringstream cmd;

    cmd << "Exit";
    m_xpad->sendNoWait(cmd.str());

    DEB_TRACE() << "********** Outside of Camera::exit ***********";
}













