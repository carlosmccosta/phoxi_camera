//
// Created by controller on 1/11/18.
//

#include "phoxi_camera/PhoXiInterface.h"

PhoXiInterface::PhoXiInterface() : minIntensity(0.0f), maxIntensity(0.0f) {

}

std::vector<std::string> PhoXiInterface::cameraList(){
    if (!phoXiFactory.isPhoXiControlRunning()){
        scanner.Reset();
        throw PhoXiControlNotRunning("PhoXi Control is not running");
    }
    std::vector<std::string> list;
    auto DeviceList = phoXiFactory.GetDeviceList();
    for (int i = 0; i < DeviceList.size(); ++i) {
        list.push_back(DeviceList[i].HWIdentification);
    }
    return list;
}

void PhoXiInterface::connectCamera(std::string HWIdentification, pho::api::PhoXiTriggerMode mode, bool startAcquisition){
    if(this->isConnected()){
        if(scanner->HardwareIdentification == HWIdentification){
            this->setTriggerMode(mode,startAcquisition);
            return;
        }
    }
    if (!phoXiFactory.isPhoXiControlRunning()) {
        throw PhoXiControlNotRunning("PhoXi Control is not running");
    }
    auto DeviceList = phoXiFactory.GetDeviceList();
    bool found = false;
    std::string device;
    for(int i= 0; i< DeviceList.size(); i++){
        if(DeviceList[i].HWIdentification == HWIdentification) {
            found = true;
            device = DeviceList[i].HWIdentification;
            break;
        }
    }
    if(!found){
        throw PhoXiScannerNotFound("Scanner not found");
    }
    disconnectCamera();
    if(!(scanner = phoXiFactory.CreateAndConnect(device,5000))){
        disconnectCamera();
        throw UnableToStartAcquisition("Scanner was not able to connect. Disconnected.");
    }
    this->setTriggerMode(mode,startAcquisition);
    return ;
}
void PhoXiInterface::disconnectCamera(){
    if(scanner && scanner->isConnected()){
        scanner->Disconnect(true);
    }
}

pho::api::PFrame PhoXiInterface::getPFrame(int id){
    if(id < 0){
        id = this->triggerImage();
    }
    this->isOk();
    return  scanner->GetSpecificFrame(id,10000);
}

std::shared_ptr<pcl::PointCloud<pcl::PointXYZRGBNormal>> PhoXiInterface::getPointCloud() {
    return getPointCloudFromFrame(getPFrame(-1));
}

std::shared_ptr<pcl::PointCloud<pcl::PointXYZRGBNormal>> PhoXiInterface::getPointCloudFromFrame(pho::api::PFrame frame) {
    if (!frame || !frame->Successful) {
        throw CorruptedFrame("Corrupted frame!");
    }
    bool autoMinMaxIntensityUsed = false;
    bool textureAvailable = scanner->OutputSettings->SendTexture && !frame->Texture.Empty();
    if (textureAvailable) {
        if (minIntensity < 0.0f || maxIntensity <= 0.0f || minIntensity >= maxIntensity) {
            minIntensity = std::numeric_limits<float>::max();
            maxIntensity = std::numeric_limits<float>::min();
            autoMinMaxIntensityUsed = true;
            for(int r = 0; r < frame->Texture.Size.Height; r++) {
                for (int c = 0; c < frame->Texture.Size.Width; c++) {
                    auto point = frame->PointCloud.At(r,c);
                    if (point.x > 0 && point.y > 0 && point.z > 0) {
                        float intensity = frame->Texture.At(r,c);
                        if (intensity > maxIntensity)
                            maxIntensity = intensity;
                        if (intensity < minIntensity)
                            minIntensity = intensity;
                    }
                }
            }
        }
    }
    std::shared_ptr<pcl::PointCloud<pcl::PointXYZRGBNormal>> cloud(new pcl::PointCloud<pcl::PointXYZRGBNormal>(frame->GetResolution().Width,frame->GetResolution().Height));
    for(int r = 0; r < frame->GetResolution().Height; r++){
        for (int c = 0; c < frame->GetResolution().Width; c++){
            auto point = frame->PointCloud.At(r,c);
            auto normal = frame->NormalMap.At(r,c);
            uint8_t intensity8Bits = 0;
            if (textureAvailable) {
                float intensityMaxTruncated = std::min((float)frame->Texture.At(r,c), maxIntensity);
                float intensityMinTruncated = std::max(intensityMaxTruncated, minIntensity);
                intensity8Bits = (uint8_t)((intensityMinTruncated - minIntensity) / (maxIntensity - minIntensity) * std::numeric_limits<uint8_t>::max());
            }
            pcl::PointXYZRGBNormal pclPoint;
            pclPoint.x = point.x / 1000.0;
            pclPoint.y = point.y / 1000.0;
            pclPoint.z = point.z / 1000.0;
            pclPoint.normal_x = normal.x;
            pclPoint.normal_y = normal.y;
            pclPoint.normal_z = normal.z;
            pclPoint.r = intensity8Bits;
            pclPoint.g = intensity8Bits;
            pclPoint.b = intensity8Bits;
            cloud->at(c,r) = pclPoint;
        }
    }
    if (autoMinMaxIntensityUsed) {
        minIntensity = 0.0f;
        maxIntensity = 0.0f;
    }
    return cloud;
}

void PhoXiInterface::isOk(){
    if(!scanner || !scanner->isConnected()){
        throw PhoXiScannerNotConnected("No scanner connected");
    }
    return;
}

void PhoXiInterface::setCoordinateSpace(pho::api::PhoXiCoordinateSpace space){
    this->isOk();
    scanner->CoordinatesSettings->CoordinateSpace = space;
    return;
}

pho::api::PhoXiCoordinateSpace PhoXiInterface::getCoordinateSpace(){
    this->isOk();
    return scanner->CoordinatesSettings->CoordinateSpace;
}

void PhoXiInterface::setTransformation(pho::api::PhoXiCoordinateTransformation coordinateTransformation,pho::api::PhoXiCoordinateSpace space,bool setSpace = true, bool saveSettings = true){
    this->isOk();
    pho::api::PhoXiCoordinatesSettings settings = scanner->CoordinatesSettings;
    switch(space){
        case pho::api::PhoXiCoordinateSpace::RobotSpace:
            if(!settings.RobotTransformation.isSupported()){
                throw CoordinationSpaceNotSupported("Coordination space is not supported");
            }
            settings.RobotTransformation = coordinateTransformation;
            settings.CoordinateSpace = pho::api::PhoXiCoordinateSpace::RobotSpace;
            break;
        case pho::api::PhoXiCoordinateSpace::CustomSpace:
            if(!settings.CustomTransformation.isSupported()){
                throw CoordinationSpaceNotSupported("Coordination space is not supported");
            }
            settings.CustomTransformation = coordinateTransformation;
            settings.CoordinateSpace = pho::api::PhoXiCoordinateSpace::CustomSpace;
            break;
        default:
            throw CoordinationSpaceNotSupported("Coordination space is not supported");
    }
    if(setSpace){
        settings.CoordinateSpace = space;
    }
    this->isOk();
    scanner->CoordinatesSettings = settings;
    if(saveSettings){
        scanner->SaveSettings();
    }
}

std::string PhoXiInterface::getHardwareIdentification(){
    this->isOk();
    return scanner->HardwareIdentification;
}

bool PhoXiInterface::isConnected(){
    return (scanner && scanner->isConnected());
}
bool PhoXiInterface::isAcquiring(){
    return (scanner && scanner->isAcquiring());
}
void PhoXiInterface::startAcquisition(){
    this->isOk();
    if(scanner->isAcquiring()){
        return;
    }
    scanner->StartAcquisition();
    if(!scanner->isAcquiring()){
        throw UnableToStartAcquisition("Unable to start acquisition.");
    }
}
void PhoXiInterface::stopAcquisition(){
    this->isOk();
    if(!scanner->isAcquiring()){
        return;
    }
    scanner->StopAcquisition();
    if(scanner->isAcquiring()){
        throw UnableToStopAcquisition("Unable to stop acquisition.");
    }
}
int PhoXiInterface::triggerImage(){
    this->setTriggerMode(pho::api::PhoXiTriggerMode::Software,true);
    return scanner->TriggerFrame();
}

std::vector<pho::api::PhoXiCapturingMode> PhoXiInterface::getSupportedCapturingModes(){
    this->isOk();
    return scanner->SupportedCapturingModes;
}

void PhoXiInterface::setHighResolution(){
    this->isOk();
    pho::api::PhoXiCapturingMode mode = scanner->CapturingMode;
    mode.Resolution.Width = 2064;
    mode.Resolution.Height = 1544;
    scanner->CapturingMode = mode;
}
void PhoXiInterface::setLowResolution(){
    this->isOk();
    pho::api::PhoXiCapturingMode mode = scanner->CapturingMode;
    mode.Resolution.Width = 1032;
    mode.Resolution.Height = 772;
    scanner->CapturingMode = mode;
}

void PhoXiInterface::setTriggerMode(pho::api::PhoXiTriggerMode mode, bool startAcquisition){
    if(!((mode == pho::api::PhoXiTriggerMode::Software) || (mode == pho::api::PhoXiTriggerMode::Hardware) || (mode == pho::api::PhoXiTriggerMode::Freerun) || (mode == pho::api::PhoXiTriggerMode::NoValue))){
        throw InvalidTriggerMode("Invalid trigger mode " + std::to_string(mode) +".");
    }
    this->isOk();
    if(mode == scanner->TriggerMode.GetValue()){
        if(startAcquisition){
            this->startAcquisition();
        }
        else{
            this->stopAcquisition();
        }
        return;
    }
    this->stopAcquisition();
    scanner->TriggerMode = mode;
    if(startAcquisition){
        this->startAcquisition();
    }
}

pho::api::PhoXiTriggerMode PhoXiInterface::getTriggerMode(){
    this->isOk();
    return scanner->TriggerMode;
}
