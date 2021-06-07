//
// Created by controller on 1/11/18.
//

#include "phoxi_camera/PhoXiInterface.h"
#include <opencv2/imgproc.hpp>

PhoXiInterface::PhoXiInterface() :
        textureMinIntensity(0.0f),
        textureMaxIntensity(0.0f),
        textureContrastLimitedAdaptiveHistogramEqualizationClipLimit(4.0),
        textureContrastLimitedAdaptiveHistogramEqualizationSizeX(4),
        textureContrastLimitedAdaptiveHistogramEqualizationSizeY(4),
        generatePointCloudWithOnlyValidPoints(false) {}

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
}
void PhoXiInterface::disconnectCamera(){
    if(scanner && scanner->isConnected()){
        scanner->Disconnect(true);
    }
}

PFramePostProcessed PhoXiInterface::getPFrame(int id){
    if(id < 0){
        id = this->triggerImage();
    }
    this->isOk();
    return postProcessFrame(scanner->GetSpecificFrame(id,10000));
}

PFramePostProcessed PhoXiInterface::postProcessFrame(pho::api::PFrame frame) {
    auto frameProcessed = PFramePostProcessed(new FramePostProcessed());
    frameProcessed->PFrame = frame;
    bool textureAvailable = scanner->OutputSettings->SendTexture && !frameProcessed->PFrame->Texture.Empty();
    if (textureAvailable) {
        frameProcessed->TextureAfterPostProcessing = cv::Mat(frameProcessed->PFrame->Texture.Size.Height, frameProcessed->PFrame->Texture.Size.Width, CV_32FC1, frameProcessed->PFrame->Texture.operator[](0));
        bool useAutoMinMax = textureMinIntensity < 0.0f || textureMaxIntensity <= 0.0f || textureMinIntensity >= textureMaxIntensity;
        if (useAutoMinMax) {
            cv::normalize(frameProcessed->TextureAfterPostProcessing, frameProcessed->TextureAfterPostProcessing, 0, 255, cv::NORM_MINMAX, CV_8U);
        } else {
            double scale = (std::numeric_limits<uint8_t>::max()) * (1.0 / (textureMaxIntensity - textureMinIntensity));
            double shift = textureMinIntensity - (textureMinIntensity * scale);
            frameProcessed->TextureAfterPostProcessing.convertTo(frameProcessed->TextureAfterPostProcessing, CV_8U, scale, shift);
        }
        bool useTextureHistogramEqualization = textureContrastLimitedAdaptiveHistogramEqualizationSizeX > 0 && textureContrastLimitedAdaptiveHistogramEqualizationSizeY > 0;
        if (useTextureHistogramEqualization) {
            auto clahe = cv::createCLAHE(textureContrastLimitedAdaptiveHistogramEqualizationClipLimit, cv::Size(textureContrastLimitedAdaptiveHistogramEqualizationSizeX, textureContrastLimitedAdaptiveHistogramEqualizationSizeY));
            clahe->apply(frameProcessed->TextureAfterPostProcessing, frameProcessed->TextureAfterPostProcessing);
        }
    }
    return frameProcessed;
}

std::shared_ptr<pcl::PointCloud<pcl::PointXYZRGBNormal>> PhoXiInterface::getPointCloud() {
    return getPointCloudFromFrame(getPFrame(-1));
}

std::shared_ptr<pcl::PointCloud<pcl::PointXYZRGBNormal>> PhoXiInterface::getPointCloudFromFrame(PFramePostProcessed frame) {
    if (!frame || !frame->PFrame || !frame->PFrame->Successful) {
        throw CorruptedFrame("Corrupted frame!");
    }
    bool textureAvailable = !frame->TextureAfterPostProcessing.empty();
    bool normalMapAvailable = scanner->OutputSettings->SendNormalMap && !frame->PFrame->NormalMap.Empty();
    std::shared_ptr<pcl::PointCloud<pcl::PointXYZRGBNormal>> cloud;
    if (generatePointCloudWithOnlyValidPoints)
        cloud = std::shared_ptr< pcl::PointCloud<pcl::PointXYZRGBNormal> >(new pcl::PointCloud<pcl::PointXYZRGBNormal>());
    else
        cloud = std::shared_ptr< pcl::PointCloud<pcl::PointXYZRGBNormal> >(new pcl::PointCloud<pcl::PointXYZRGBNormal>(frame->PFrame->GetResolution().Width,frame->PFrame->GetResolution().Height));
    for(int r = 0; r < frame->PFrame->GetResolution().Height; r++){
        for (int c = 0; c < frame->PFrame->GetResolution().Width; c++){
            auto point = frame->PFrame->PointCloud.At(r,c);
            bool validPoint = point != PhoXiInterface::invalidPoint;
            if (!generatePointCloudWithOnlyValidPoints ||
                (generatePointCloudWithOnlyValidPoints && validPoint)) {
                pcl::PointXYZRGBNormal pclPoint;
                if (validPoint) {
                    pclPoint.x = point.x / 1000.0;
                    pclPoint.y = point.y / 1000.0;
                    pclPoint.z = point.z / 1000.0;
                    if (normalMapAvailable) {
                        auto normal = frame->PFrame->NormalMap.At(r,c);
                        pclPoint.normal_x = normal.x;
                        pclPoint.normal_y = normal.y;
                        pclPoint.normal_z = normal.z;
                    }
                    if (textureAvailable) {
                        uint8_t intensity = frame->TextureAfterPostProcessing.at<uint8_t>(r,c);
                        pclPoint.r = intensity;
                        pclPoint.g = intensity;
                        pclPoint.b = intensity;
                    }
                } else {
                    pclPoint.x = std::numeric_limits<float>::quiet_NaN();
                    pclPoint.y = std::numeric_limits<float>::quiet_NaN();
                    pclPoint.z = std::numeric_limits<float>::quiet_NaN();
                }
                if (generatePointCloudWithOnlyValidPoints)
                    cloud->push_back(pclPoint);
                else
                    cloud->at(c,r) = pclPoint;
            }
        }
    }
    cloud->is_dense = generatePointCloudWithOnlyValidPoints;
    return cloud;
}

void PhoXiInterface::isOk(){
    if(!scanner || !scanner->isConnected()){
        throw PhoXiScannerNotConnected("No scanner connected");
    }
}

void PhoXiInterface::setCoordinateSpace(pho::api::PhoXiCoordinateSpace space){
    this->isOk();
    scanner->CoordinatesSettings->CoordinateSpace = space;
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

static const pho::api::Point3_32f PhoXiInterface::invalidPoint = pho::api::Point3_32f(0.0f, 0.0f, 0.0f);
