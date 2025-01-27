//
// Created by controller on 1/11/18.
//

#include "phoxi_camera/RosInterface.h"

namespace phoxi_camera {
    RosInterface::RosInterface() : nh("~"), dynamicReconfigureServer(dynamicReconfigureMutex, nh),
                                   PhoXi3DscannerDiagnosticTask("PhoXi3Dscanner",
                                                                boost::bind(&RosInterface::diagnosticCallback, this,
                                                                            _1)) {

        //create service servers
        getDeviceListService = nh.advertiseService("get_device_list", &RosInterface::getDeviceList, this);
        connectCameraService =nh.advertiseService("connect_camera", &RosInterface::connectCamera, this);
        isConnectedService = nh.advertiseService("is_connected", (bool (RosInterface::*)(phoxi_camera::IsConnected::Request&, phoxi_camera::IsConnected::Response&))&RosInterface::isConnected, this);
        isAcquiringService = nh.advertiseService("is_acquiring", (bool (RosInterface::*)(phoxi_camera::IsAcquiring::Request&, phoxi_camera::IsAcquiring::Response&))&RosInterface::isAcquiring, this);
        isConnectedServiceV2 = nh.advertiseService("V2/is_connected", (bool (RosInterface::*)(phoxi_camera::GetBool::Request&, phoxi_camera::GetBool::Response&))&RosInterface::isConnected, this);
        isAcquiringServiceV2 = nh.advertiseService("V2/is_acquiring", (bool (RosInterface::*)(phoxi_camera::GetBool::Request&, phoxi_camera::GetBool::Response&))&RosInterface::isAcquiring, this);
        startAcquisitionService = nh.advertiseService("start_acquisition", (bool (RosInterface::*)(std_srvs::Empty::Request&, std_srvs::Empty::Response&))&RosInterface::startAcquisition, this);
        stopAcquisitionService = nh.advertiseService("stop_acquisition", (bool (RosInterface::*)(std_srvs::Empty::Request&, std_srvs::Empty::Response&))&RosInterface::stopAcquisition, this);
        startAcquisitionServiceV2 = nh.advertiseService("V2/start_acquisition", (bool (RosInterface::*)(phoxi_camera::Empty::Request&, phoxi_camera::Empty::Response&))&RosInterface::startAcquisition, this);
        stopAcquisitionServiceV2 = nh.advertiseService("V2/stop_acquisition", (bool (RosInterface::*)(phoxi_camera::Empty::Request&, phoxi_camera::Empty::Response&))&RosInterface::stopAcquisition, this);
        triggerImageService =nh.advertiseService("trigger_image", &RosInterface::triggerImage, this);
        getFrameService = nh.advertiseService("get_frame", &RosInterface::getFrame, this);
        saveFrameService = nh.advertiseService("save_frame", &RosInterface::saveFrame, this);
        getScanVolumeService = nh.advertiseService("get_scanning_volume", &RosInterface::getScanningVolume, this);
        disconnectCameraService = nh.advertiseService("disconnect_camera", &RosInterface::disconnectCamera, this);
        getHardwareIdentificationService = nh.advertiseService("get_hardware_indentification", &RosInterface::getHardwareIdentification, this);
        getSupportedCapturingModesService = nh.advertiseService("get_supported_capturing_modes", &RosInterface::getSupportedCapturingModes, this);
        getApiVersionService = nh.advertiseService("get_api_version", &RosInterface::getApiVersion, this);
        getFirmwareVersionService = nh.advertiseService("get_firmware_version", &RosInterface::getFirmwareVersion, this);
#ifndef PHOXI_API_v1_1
        setCoordianteSpaceService = nh.advertiseService("V2/set_transformation",&RosInterface::setTransformation, this);
        setTransformationService = nh.advertiseService("V2/set_coordinate_space",&RosInterface::setCoordianteSpace, this);
        saveLastFrameService = nh.advertiseService("V2/save_last_frame", &RosInterface::saveLastFrame, this);
#endif

        //create publishers
        bool latch_topics;
        int topic_queue_size;
        nh.param<bool>("latch_topics", latch_topics, false);
        nh.param<int>("topic_queue_size", topic_queue_size, 1);
        cloudPub = nh.advertise<sensor_msgs::PointCloud2>("pointcloud", 1, latch_topics);
        normalMapPub = nh.advertise<sensor_msgs::Image>("normal_map", topic_queue_size, latch_topics);
        confidenceMapPub = nh.advertise<sensor_msgs::Image>("confidence_map", topic_queue_size, latch_topics);
        rawTexturePub = nh.advertise<sensor_msgs::Image>("texture", topic_queue_size, latch_topics);
        rgbTexturePub = nh.advertise<sensor_msgs::Image>("rgb_texture", topic_queue_size, latch_topics);
        rawColorCameraPub = nh.advertise<sensor_msgs::Image>("color_camera", topic_queue_size, latch_topics);
        colorCameraPub = nh.advertise<sensor_msgs::Image>("rgb_color_camera", topic_queue_size, latch_topics);
        depthMapPub = nh.advertise<sensor_msgs::Image>("depth_map", topic_queue_size, latch_topics);
        cameraInfoPub = nh.advertise<sensor_msgs::CameraInfo>("texture_camera_info", topic_queue_size, latch_topics);
        cameraInfoColorCameraPub = nh.advertise<sensor_msgs::CameraInfo>("color_camera_camera_info", topic_queue_size, latch_topics);

        //set diagnostic Hw id
        diagnosticUpdater.setHardwareID("none");
        diagnosticUpdater.add(PhoXi3DscannerDiagnosticTask);
        diagnosticTimer = nh.createTimer(ros::Duration(5.0), &RosInterface::diagnosticTimerCallback, this);
        diagnosticTimer.start();

        nh.param<std::string>("frame_id", frameId, "PhoXi3Dscanner_sensor");

        //connect to default scanner
        std::string scannerId;
        if (nh.param<std::string>("scanner_id", scannerId, "") && !scannerId.empty()) {
            try {
                RosInterface::connectCamera(scannerId);
                ROS_INFO("Connected to %s", scannerId.c_str());
            } catch (PhoXiInterfaceException& e) {
                ROS_WARN("Connection to default scanner %s failed. %s ", scannerId.c_str(), e.what());
            }
        }
        if (!PhoXiInterface::isConnected()) {
            getDefaultDynamicReconfigureConfig(dynamicReconfigureConfig);
        }
        //set dynamic reconfigure callback
        dynamicReconfigureServer.setCallback(boost::bind(&RosInterface::dynamicReconfigureCallback, this, _1, _2));
    }

    bool
    RosInterface::getDeviceList(phoxi_camera::GetDeviceList::Request& req, phoxi_camera::GetDeviceList::Response& res) {
        try {
            res.out = PhoXiInterface::cameraList();
            phoXiDeviceInforamtionToRosMsg(PhoXiInterface::deviceList(), res.device_information_list);
            res.len = res.out.size();
            res.success = true;
            res.message = OKRESPONSE;
        } catch (PhoXiInterfaceException& e) {
            res.success = false;
            res.message = e.what();
        }
        return true;
    }

    bool
    RosInterface::connectCamera(phoxi_camera::ConnectCamera::Request& req, phoxi_camera::ConnectCamera::Response& res) {
        try {
            RosInterface::connectCamera(req.name);
            res.success = true;
            res.message = OKRESPONSE;
        } catch (PhoXiInterfaceException& e) {
            res.success = false;
            res.message = e.what();
        }
        return true;
    }

    bool RosInterface::isConnected(phoxi_camera::IsConnected::Request& req, phoxi_camera::IsConnected::Response& res) {
        res.connected = PhoXiInterface::isConnected();
        return true;
    }

    bool RosInterface::isAcquiring(phoxi_camera::IsAcquiring::Request& req, phoxi_camera::IsAcquiring::Response& res) {
        res.is_acquiring = PhoXiInterface::isAcquiring();
        return true;
    }

    bool RosInterface::isConnected(phoxi_camera::GetBool::Request& req, phoxi_camera::GetBool::Response& res) {
        res.value = PhoXiInterface::isConnected();
        res.message = OKRESPONSE; //todo tot este premysliet
        res.success = true;
        return true;
    }

    bool RosInterface::isAcquiring(phoxi_camera::GetBool::Request& req, phoxi_camera::GetBool::Response& res) {
        res.value = PhoXiInterface::isAcquiring();
        res.message = OKRESPONSE; //todo tot este premysliet
        res.success = true;
        return true;
    }

    bool RosInterface::startAcquisition(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res) {
        try {
            PhoXiInterface::startAcquisition();
            dynamicReconfigureConfig.start_acquisition = true;
            dynamicReconfigureServer.updateConfig(dynamicReconfigureConfig);
            diagnosticUpdater.force_update();
        } catch (PhoXiInterfaceException& e) {
            ROS_ERROR("%s", e.what());
        }
        return true;
    }

    bool RosInterface::stopAcquisition(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res) {
        try {
            PhoXiInterface::stopAcquisition();
            dynamicReconfigureConfig.start_acquisition = false;
            dynamicReconfigureServer.updateConfig(dynamicReconfigureConfig);
            diagnosticUpdater.force_update();
        } catch (PhoXiInterfaceException& e) {
            ROS_ERROR("%s", e.what());
        }
        return true;
    }

    bool RosInterface::startAcquisition(phoxi_camera::Empty::Request& req, phoxi_camera::Empty::Response& res) {
        try {
            dynamicReconfigureConfig.start_acquisition = true;
            dynamicReconfigureServer.updateConfig(dynamicReconfigureConfig);
            PhoXiInterface::startAcquisition();
            res.message = OKRESPONSE;
            res.success = true;
        } catch (PhoXiInterfaceException& e) {
            res.message = e.what();
            res.success = false;
        }
        return true;
    }

    bool RosInterface::stopAcquisition(phoxi_camera::Empty::Request& req, phoxi_camera::Empty::Response& res) {
        try {
            dynamicReconfigureConfig.start_acquisition = false;
            dynamicReconfigureServer.updateConfig(dynamicReconfigureConfig);
            PhoXiInterface::stopAcquisition();
            res.message = OKRESPONSE;
            res.success = true;
        } catch (PhoXiInterfaceException& e) {
            res.message = e.what();
            res.success = false;
        }
        return true;
    }

    bool
    RosInterface::triggerImage(phoxi_camera::TriggerImage::Request& req, phoxi_camera::TriggerImage::Response& res) {
        try {
            res.id = RosInterface::triggerImage();
            res.success = true;
            res.message = OKRESPONSE;
        } catch (PhoXiInterfaceException& e) {
            res.success = false;
            res.message = e.what();
        }
        return true;
    }

    bool RosInterface::getFrame(phoxi_camera::GetFrame::Request& req, phoxi_camera::GetFrame::Response& res) {
        try {
            pho::api::PFrame frame = getPFrame(req.in);
            publishFrame(frame);
            if (!frame) {
                res.success = false;
                res.message = "Null frame!";
            } else {
                res.success = true;
                res.message = OKRESPONSE;
            }
        } catch (PhoXiInterfaceException& e) {
            res.success = false;
            res.message = e.what();
        }
        return true;
    }

    bool RosInterface::saveFrame(phoxi_camera::SaveFrame::Request& req, phoxi_camera::SaveFrame::Response& res) {
        try {
            pho::api::PFrame frame = RosInterface::getPFrame(req.in);
            if (!frame) {
                res.success = false;
                res.message = "Null frame!";
                return true;
            }
            size_t pos = req.path.find("~");
            if (pos != std::string::npos) {
                char* home = std::getenv("HOME");
                if (!home) {
                    res.message = "'~' found in 'path' parameter but environment variable 'HOME' not found. Export' HOME' variable or pass absolute value to 'path' parameter.";
                    res.success = false;
                    return true;
                }
                req.path.replace(pos, 1, home);
            }
            ROS_INFO("path: %s", req.path.c_str());
            frame->SaveAsPly(req.path);
            res.message = OKRESPONSE;
            res.success = true;
        } catch (PhoXiInterfaceException& e) {
            res.success = false;
            res.message = e.what();
        }
        return true;
    }

    bool RosInterface::getScanningVolume(phoxi_camera::GetScanningVolume::Request& req, phoxi_camera::GetScanningVolume::Response& res) {
        try {
            pho::api::PhoXiScanningVolume scanVolume = scanner->ScanningVolume;
            pho::api::PhoXiMesh phoxiMesh = scanVolume.Mesh;
            pcl::PolygonMesh meshPcl;

            res.success = fromPhoxiMeshToPolygonMesh(phoxiMesh, meshPcl);
            pcl_conversions::moveFromPCL(meshPcl, res.mesh);
            res.message = OKRESPONSE;
        } catch (PhoXiInterfaceException& e) {
            res.success = false;
            res.message = e.what();
        }
        return true;
    }

    bool RosInterface::disconnectCamera(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res) {
        try {
            PhoXiInterface::disconnectCamera();
            diagnosticUpdater.force_update();
        } catch (PhoXiInterfaceException& e) {
            //scanner is already disconnected on exception
        }
        return true;
    }

    bool RosInterface::getHardwareIdentification(phoxi_camera::GetHardwareIdentification::Request& req,
                                                 phoxi_camera::GetHardwareIdentification::Response& res) {
        try {
            res.hardware_identification = PhoXiInterface::getHardwareIdentification();
            res.success = true;
            res.message = OKRESPONSE;
        } catch (PhoXiInterfaceException& e) {
            res.success = false;
            res.message = e.what();
        }
        return true;
    }

    bool RosInterface::getSupportedCapturingModes(phoxi_camera::GetSupportedCapturingModes::Request& req,
                                                  phoxi_camera::GetSupportedCapturingModes::Response& res) {
        try {
            std::vector<pho::api::PhoXiCapturingMode> modes = PhoXiInterface::getSupportedCapturingModes();
            for (int i = 0; i < modes.size(); i++) {
                phoxi_camera::PhoXiSize size;
                size.Height = modes[i].Resolution.Height;
                size.Width = modes[i].Resolution.Width;
                res.supported_capturing_modes.push_back(size);
            }
            res.success = true;
            res.message = OKRESPONSE;
        } catch (PhoXiInterfaceException& e) {
            res.success = false;
            res.message = e.what();
        }
        return true;
    }

    bool RosInterface::getApiVersion(phoxi_camera::GetString::Request& req, phoxi_camera::GetString::Response& res) {
        res.value = PhoXiInterface::getApiVersion();
        res.success = true;
        return true;
    }

    bool
    RosInterface::getFirmwareVersion(phoxi_camera::GetString::Request& req, phoxi_camera::GetString::Response& res) {
        try {
            auto dl = PhoXiInterface::deviceList();
            auto it = std::find(dl.begin(), dl.end(), PhoXiInterface::getHardwareIdentification());
            if (it != dl.end()) {
                res.value = it->firmwareVersion;
            }
        }
        catch (phoxi_camera::PhoXiInterfaceException& e) {
            res.message = e.what();
            res.success = false;
        }
        res.success = true;
        return true;
    }

    void RosInterface::publishFrame(pho::api::PFrame frame) {
        if (!frame) {
            ROS_WARN("NUll frame!");
            return;
        }

        ros::Time timeNow = ros::Time::now();

        std_msgs::Header header;
        header.stamp = timeNow;
        header.frame_id = frameId;
        header.seq = frame->Info.FrameIndex;

        if (frame->PointCloud.Empty()) {
            ROS_WARN("Empty point cloud!");
        } else {
            auto cloud = PhoXiInterface::getPointCloudFromFrame(frame, dynamicReconfigureConfig.organized_cloud);
            sensor_msgs::PointCloud2 output_cloud;
            pcl::toROSMsg(*cloud, output_cloud);
            output_cloud.header = header;
            cloudPub.publish(output_cloud);
        }

        if (frame->DepthMap.Empty()) {
            ROS_WARN("Empty depth map!");
        } else {
            sensor_msgs::Image depth_map;
            depth_map.header = header;
            depth_map.encoding = sensor_msgs::image_encodings::TYPE_32FC1;
            sensor_msgs::fillImage(depth_map,
                                   sensor_msgs::image_encodings::TYPE_32FC1,
                                   frame->DepthMap.Size.Height, // height
                                   frame->DepthMap.Size.Width, // width
                                   frame->DepthMap.Size.Width * sizeof(float), // stepSize
                                   frame->DepthMap.operator[](0));
            depthMapPub.publish(depth_map);
        }

        if (!(frame->TextureRGB.Empty())) {
            sensor_msgs::Image texture;
            texture.header = header;
            texture.encoding = sensor_msgs::image_encodings::TYPE_16UC3;
            sensor_msgs::fillImage(texture, sensor_msgs::image_encodings::TYPE_16UC3,
                                   frame->TextureRGB.Size.Height, // height
                                   frame->TextureRGB.Size.Width, // width
                                   frame->TextureRGB.Size.Width * sizeof(uint16_t), // stepSize
                                   frame->TextureRGB.operator[](0));
            rawTexturePub.publish(texture);
            cv::Mat cvRgbTexture(frame->TextureRGB.Size.Height, frame->TextureRGB.Size.Width, CV_16UC3, frame->TextureRGB.operator[](0));
            cv::normalize(cvRgbTexture, cvRgbTexture, 0, 255, CV_MINMAX);
            cvRgbTexture.convertTo(cvRgbTexture, CV_8UC3);
            cv::cvtColor(cvRgbTexture, cvRgbTexture, CV_RGB2HSV);
            cv::Mat cvHSVChannelsSplit[3];
            cv::split(cvRgbTexture, cvHSVChannelsSplit);
            cv::equalizeHist(cvHSVChannelsSplit[2], cvHSVChannelsSplit[2]);
            cv::merge(cvHSVChannelsSplit, 3, cvRgbTexture);
            cv::cvtColor(cvRgbTexture, cvRgbTexture, CV_HSV2RGB);
            cv_bridge::CvImage rgbTexture(header, sensor_msgs::image_encodings::RGB8, cvRgbTexture);
            rgbTexturePub.publish(rgbTexture.toImageMsg());

            sensor_msgs::CameraInfo info;
            info.header = header;
            info.width = texture.width;
            info.height = texture.height;
            info.distortion_model = sensor_msgs::distortion_models::PLUMB_BOB;
            for (std::size_t r = 0; r < 3; ++r) {
              for (std::size_t c = 0; c < 3; ++c) {
                info.K[c + r * 3] = scanner->CalibrationSettings->CameraMatrix.At(r, c);
                info.P[c + r * 4] = scanner->CalibrationSettings->CameraMatrix.At(r, c);
                if (r == c)
                  info.R[c + r * 3] = 1.0;
              }
            }
            info.D = scanner->CalibrationSettings->DistortionCoefficients;
            cameraInfoPub.publish(info);
        } else if (!(frame->Texture.Empty())) {
            sensor_msgs::Image texture;
            texture.header = header;
            texture.encoding = sensor_msgs::image_encodings::TYPE_32FC1;
            sensor_msgs::fillImage(texture, sensor_msgs::image_encodings::TYPE_32FC1,
                                   frame->Texture.Size.Height, // height
                                   frame->Texture.Size.Width, // width
                                   frame->Texture.Size.Width * sizeof(float), // stepSize
                                   frame->Texture.operator[](0));
            rawTexturePub.publish(texture);
            cv::Mat cvGreyTexture(frame->Texture.Size.Height, frame->Texture.Size.Width, CV_32FC1,
                                  frame->Texture.operator[](0));
            cv::normalize(cvGreyTexture, cvGreyTexture, 0, 255, CV_MINMAX);
            cvGreyTexture.convertTo(cvGreyTexture, CV_8U);
            cv::equalizeHist(cvGreyTexture, cvGreyTexture);
            cv::Mat cvRgbTexture;
            cv::cvtColor(cvGreyTexture, cvRgbTexture, CV_GRAY2RGB);
            cv_bridge::CvImage rgbTexture(header, sensor_msgs::image_encodings::RGB8, cvRgbTexture);
            rgbTexturePub.publish(rgbTexture.toImageMsg());

            sensor_msgs::CameraInfo info;
            info.header = header;
            info.width = texture.width;
            info.height = texture.height;
            info.distortion_model = sensor_msgs::distortion_models::PLUMB_BOB;
            for (std::size_t r = 0; r < 3; ++r) {
              for (std::size_t c = 0; c < 3; ++c) {
                info.K[c + r * 3] = scanner->CalibrationSettings->CameraMatrix.At(r, c);
                info.P[c + r * 4] = scanner->CalibrationSettings->CameraMatrix.At(r, c);
                if (r == c)
                  info.R[c + r * 3] = 1.0;
              }
            }
            info.D = scanner->CalibrationSettings->DistortionCoefficients;
            cameraInfoPub.publish(info);
        } else {
            ROS_WARN("Empty texture!");
        }

        if (frame->ColorCameraImage.Empty()) {
            ROS_WARN("Empty ColorCameraImage!");
        } else {
            sensor_msgs::Image texture;
            texture.header.stamp = header.stamp;
            texture.header.frame_id = frameId + "_color";
            texture.encoding = sensor_msgs::image_encodings::TYPE_16UC3;
            sensor_msgs::fillImage(texture, sensor_msgs::image_encodings::TYPE_16UC3,
                                   frame->ColorCameraImage.Size.Height, // height
                                   frame->ColorCameraImage.Size.Width, // width
                                   frame->ColorCameraImage.Size.Width * sizeof(uint16_t), // stepSize
                                   frame->ColorCameraImage.operator[](0));
            rawColorCameraPub.publish(texture);
            cv::Mat cvRgbTexture(frame->ColorCameraImage.Size.Height, frame->ColorCameraImage.Size.Width, CV_16UC3, frame->ColorCameraImage.operator[](0));
            cv::normalize(cvRgbTexture, cvRgbTexture, 0, 255, CV_MINMAX);
            cvRgbTexture.convertTo(cvRgbTexture, CV_8UC3);
            cv::cvtColor(cvRgbTexture, cvRgbTexture, CV_RGB2HSV);
            cv::Mat cvHSVChannelsSplit[3];
            cv::split(cvRgbTexture, cvHSVChannelsSplit);
            cv::equalizeHist(cvHSVChannelsSplit[2], cvHSVChannelsSplit[2]);
            cv::merge(cvHSVChannelsSplit, 3, cvRgbTexture);
            cv::cvtColor(cvRgbTexture, cvRgbTexture, CV_HSV2RGB);
            cv_bridge::CvImage rgbTexture(texture.header, sensor_msgs::image_encodings::RGB8, cvRgbTexture);
            colorCameraPub.publish(rgbTexture.toImageMsg());

            sensor_msgs::CameraInfo info;
            info.header = texture.header;
            info.width = texture.width;
            info.height = texture.height;
            info.distortion_model = sensor_msgs::distortion_models::PLUMB_BOB;
            for (std::size_t r = 0; r < 3; ++r) {
              for (std::size_t c = 0; c < 3; ++c) {
                info.K[c + r * 3] = scanner->ColorCameraCalibrationSettings->CalibrationSettings.CameraMatrix.At(r, c);
                info.P[c + r * 4] = scanner->ColorCameraCalibrationSettings->CalibrationSettings.CameraMatrix.At(r, c);
                if (r == c)
                  info.R[c + r * 3] = 1.0;
              }
            }
            info.D = scanner->ColorCameraCalibrationSettings->CalibrationSettings.DistortionCoefficients;
            cameraInfoColorCameraPub.publish(info);
        }

        if (frame->ConfidenceMap.Empty()) {
            ROS_WARN("Empty confidence map!");
        } else {
            sensor_msgs::Image confidence_map;
            confidence_map.header = header;
            confidence_map.encoding = sensor_msgs::image_encodings::TYPE_32FC1;
            sensor_msgs::fillImage(confidence_map,
                                   sensor_msgs::image_encodings::TYPE_32FC1,
                                   frame->ConfidenceMap.Size.Height, // height
                                   frame->ConfidenceMap.Size.Width, // width
                                   frame->ConfidenceMap.Size.Width * sizeof(float), // stepSize
                                   frame->ConfidenceMap.operator[](0));
            confidenceMapPub.publish(confidence_map);
        }

        if (frame->NormalMap.Empty()) {
            ROS_WARN("Empty normal map!");
        } else {
            sensor_msgs::Image normal_map;
            normal_map.header = header;
            normal_map.encoding = sensor_msgs::image_encodings::TYPE_32FC3;
            sensor_msgs::fillImage(normal_map,
                                   sensor_msgs::image_encodings::TYPE_32FC3,
                                   frame->NormalMap.Size.Height, // height
                                   frame->NormalMap.Size.Width, // width
                                   frame->NormalMap.Size.Width * sizeof(float) * 3, // stepSize
                                   frame->NormalMap.operator[](0));
            normalMapPub.publish(normal_map);
        }

    }

    void RosInterface::dynamicReconfigureCallback(phoxi_camera::phoxi_cameraConfig& config, uint32_t level) {
        if (!PhoXiInterface::isConnected()) {
            config = this->dynamicReconfigureConfig;
            ROS_WARN("Node is not connected to scanner");
            return;
        }
        try {
            if (level & (1 << 1)) {
                switch (config.resolution) {
                    case 0:
                        PhoXiInterface::setLowResolution();
                        break;
                    case 1:
                        PhoXiInterface::setHighResolution();
                        break;
                    default:
                        ROS_WARN("Resolution not supported!");
                        break;
                }
                this->dynamicReconfigureConfig.resolution = config.resolution;
            }

            if (level & (1 << 2)) {
                this->isOk();
                scanner->CapturingSettings->ScanMultiplier = config.scan_multiplier;
                this->dynamicReconfigureConfig.scan_multiplier = config.scan_multiplier;
            }

            if (level & (1 << 3)) {
                this->isOk();
                scanner->CapturingSettings->ShutterMultiplier = config.shutter_multiplier;
                this->dynamicReconfigureConfig.shutter_multiplier = config.shutter_multiplier;
            }

            if (level & (1 << 4)) {
                PhoXiInterface::setTriggerMode(config.trigger_mode, config.start_acquisition);
                this->dynamicReconfigureConfig.trigger_mode = config.trigger_mode;
                this->dynamicReconfigureConfig.start_acquisition = config.start_acquisition;
            }

            if (level & (1 << 5)) {
                this->isOk();
                scanner->Timeout = config.timeout;
                this->dynamicReconfigureConfig.timeout = config.timeout;
            }

            if (level & (1 << 6)) {
                this->isOk();
                scanner->ProcessingSettings->Confidence = config.confidence;
                this->dynamicReconfigureConfig.confidence = config.confidence;
            }

            if (level & (1 << 7)) {
                this->isOk();
                scanner->OutputSettings->SendPointCloud = config.send_point_cloud;
                this->dynamicReconfigureConfig.send_point_cloud = config.send_point_cloud;
            }

            if (level & (1 << 8)) {
                this->isOk();
                scanner->OutputSettings->SendNormalMap = config.send_normal_map;
                this->dynamicReconfigureConfig.send_normal_map = config.send_normal_map;
            }

            if (level & (1 << 9)) {
                this->isOk();
                scanner->OutputSettings->SendConfidenceMap = config.send_confidence_map;
                this->dynamicReconfigureConfig.send_confidence_map = config.send_confidence_map;
            }

            if (level & (1 << 10)) {
                this->isOk();
                scanner->OutputSettings->SendTexture = config.send_texture;
                this->dynamicReconfigureConfig.send_texture = config.send_texture;
            }

            if (level & (1 << 11)) {
                this->isOk();
                scanner->OutputSettings->SendDepthMap = config.send_depth_map;
                this->dynamicReconfigureConfig.send_depth_map = config.send_depth_map;
            }

#ifndef PHOXI_API_v1_1
            if (level & (1 << 12)) {
                this->isOk();
                PhoXiInterface::setCoordinateSpace(config.coordinate_space);
                this->dynamicReconfigureConfig.coordinate_space = config.coordinate_space;
            }
#endif
            if (level & (1 << 13)) {
                this->dynamicReconfigureConfig.organized_cloud = config.organized_cloud;
            }

#ifndef PHOXI_API_v1_1
            if (level & (1 << 14)) {
                this->isOk();
                scanner->CapturingSettings->AmbientLightSuppression = config.ambient_light_suppression;
                this->dynamicReconfigureConfig.ambient_light_suppression = config.ambient_light_suppression;
            }

            if (level & (1 << 15)) {
                this->isOk();
                std::vector<double> supportedSPE = scanner->SupportedSinglePatternExposures;
                if (!supportedSPE.empty()) {    // ignore setting if setting is not supported
                    scanner->CapturingSettings->SinglePatternExposure = supportedSPE.at(config.single_pattern_exposure);
                    this->dynamicReconfigureConfig.single_pattern_exposure = config.single_pattern_exposure;
                } else {
                    ROS_WARN("Scanner setting 'Single pattern exposure' is not supported by the scanner firmware.");
                }
            }

            if (level & (1 << 16)) {
                this->isOk();
                scanner->CapturingSettings->CameraOnlyMode = config.camera_only_mode;
                this->dynamicReconfigureConfig.camera_only_mode = config.camera_only_mode;
            }
#endif

        } catch (PhoXiInterfaceException& e) {
            ROS_WARN("%s", e.what());
        }
    }

    pho::api::PFrame RosInterface::getPFrame(int id) {
        pho::api::PFrame frame = PhoXiInterface::getPFrame(id);
        //update dynamic reconfigure
        dynamicReconfigureConfig.trigger_mode = PhoXiInterface::scanner->TriggerMode.GetValue();
        dynamicReconfigureConfig.start_acquisition = PhoXiInterface::scanner->isAcquiring();
        dynamicReconfigureServer.updateConfig(dynamicReconfigureConfig);
        return frame;
    }

    int RosInterface::triggerImage() {
        int id = PhoXiInterface::triggerImage(false);
        //update dynamic reconfigure
        dynamicReconfigureConfig.coordinate_space = pho::api::PhoXiTriggerMode::Software;
        dynamicReconfigureConfig.start_acquisition = true;
        dynamicReconfigureServer.updateConfig(dynamicReconfigureConfig);
        return id;
    }

    void
    RosInterface::connectCamera(std::string HWIdentification, pho::api::PhoXiTriggerMode mode, bool startAcquisition) {
        PhoXiInterface::connectCamera(HWIdentification, mode, startAcquisition);
        bool initFromConfig = false;
        nh.getParam("init_from_config", initFromConfig);
        if (initFromConfig) {
            getDefaultDynamicReconfigureConfig(dynamicReconfigureConfig);
            this->dynamicReconfigureCallback(dynamicReconfigureConfig, std::numeric_limits<uint32_t>::max());
        } else {
            initFromPhoXi();
        }
        dynamicReconfigureServer.updateConfig(dynamicReconfigureConfig);
        diagnosticUpdater.force_update();
    }

    void RosInterface::diagnosticCallback(diagnostic_updater::DiagnosticStatusWrapper& status) {
        if (PhoXiInterface::isConnected()) {
            if (PhoXiInterface::isAcquiring()) {
                status.summary(diagnostic_msgs::DiagnosticStatus::OK, "Ready");
            } else {
                status.summary(diagnostic_msgs::DiagnosticStatus::WARN, "Acquisition not started");
            }
            status.add("HardwareIdentification", std::string(scanner->HardwareIdentification));
            status.add("Trigger mode", getTriggerMode(scanner->TriggerMode));

        } else {
            status.summary(diagnostic_msgs::DiagnosticStatus::ERROR, "Not connected");
        }
    }

    void RosInterface::diagnosticTimerCallback(const ros::TimerEvent&) {
        diagnosticUpdater.force_update();
    }

    std::string RosInterface::getTriggerMode(pho::api::PhoXiTriggerMode mode) {
        switch (mode) {
            case pho::api::PhoXiTriggerMode::Freerun:
                return "Freerun";
            case pho::api::PhoXiTriggerMode::Software:
                return "Software";
            case pho::api::PhoXiTriggerMode::Hardware:
                return "Hardware";
            case pho::api::PhoXiTriggerMode::NoValue:
                return "NoValue";
            default:
                return "Undefined";
        }
    }

    void RosInterface::initFromPhoXi() {
        getDefaultDynamicReconfigureConfig(dynamicReconfigureConfig);
        if (!scanner->isConnected()) {
            ROS_WARN("Scanner not connected.");
            return;
        }
        ///resolution
        pho::api::PhoXiCapturingMode mode = scanner->CapturingMode;
        if ((mode.Resolution.Width == 2064) && (mode.Resolution.Height = 1544)) {
            this->dynamicReconfigureConfig.resolution = 1;
        } else {
            this->dynamicReconfigureConfig.resolution = 0;
        }

        pho::api::PhoXiCapturingSettings capturingSettings;
        capturingSettings = scanner->CapturingSettings;
        this->dynamicReconfigureConfig.scan_multiplier = capturingSettings.ScanMultiplier;
        this->dynamicReconfigureConfig.shutter_multiplier = capturingSettings.ShutterMultiplier;
        this->dynamicReconfigureConfig.confidence = scanner->ProcessingSettings->Confidence;

        pho::api::FrameOutputSettings outputSettings;
        outputSettings = scanner->OutputSettings;
        this->dynamicReconfigureConfig.send_point_cloud = outputSettings.SendPointCloud;
        this->dynamicReconfigureConfig.send_normal_map = outputSettings.SendNormalMap;
        this->dynamicReconfigureConfig.send_confidence_map = outputSettings.SendConfidenceMap;
        this->dynamicReconfigureConfig.send_depth_map = outputSettings.SendDepthMap;
        this->dynamicReconfigureConfig.send_texture = outputSettings.SendTexture;
#ifndef PHOXI_API_v1_1
        this->dynamicReconfigureConfig.coordinate_space = scanner->CoordinatesSettings->CoordinateSpace;
        this->dynamicReconfigureConfig.ambient_light_suppression = capturingSettings.AmbientLightSuppression;

        std::vector<double> supportedSPE = scanner->SupportedSinglePatternExposures;
        if (!supportedSPE.empty()) {
            auto actualParam_it = std::find(supportedSPE.begin(), supportedSPE.end(), capturingSettings.SinglePatternExposure);
            if (actualParam_it != supportedSPE.end()) {
                this->dynamicReconfigureConfig.single_pattern_exposure = actualParam_it - supportedSPE.begin();
            } else {
                int singlePatternExposure_index;
                nh.getParam("single_pattern_exposure", singlePatternExposure_index);
                this->dynamicReconfigureConfig.single_pattern_exposure = singlePatternExposure_index;
                ROS_WARN("Can not update Single Pattern Exposure parameter in dynamic reconfigure, set default value from config.");
            }
        } else {
            ROS_WARN("Scanner setting 'Single pattern exposure' is not supported by the scanner firmware.");
        }

        this->dynamicReconfigureConfig.camera_only_mode = capturingSettings.CameraOnlyMode;
#endif
        this->dynamicReconfigureConfig.trigger_mode = scanner->TriggerMode.GetValue();
        this->dynamicReconfigureConfig.start_acquisition = scanner->isAcquiring();
        this->dynamicReconfigureConfig.timeout = scanner->Timeout.GetValue();
    }

#ifndef PHOXI_API_v1_1

    bool RosInterface::setCoordianteSpace(phoxi_camera::SetCoordinatesSpace::Request& req,
                                          phoxi_camera::SetCoordinatesSpace::Response& res) {
        try {
            PhoXiInterface::setCoordinateSpace(req.coordinates_space);
            //update dynamic reconfigure
            dynamicReconfigureConfig.coordinate_space = req.coordinates_space;
            dynamicReconfigureServer.updateConfig(dynamicReconfigureConfig);
            res.success = true;
            res.message = OKRESPONSE;
        } catch (PhoXiInterfaceException& e) {
            res.success = false;
            res.message = e.what();
        }
        return true;
    }

    bool RosInterface::setTransformation(phoxi_camera::SetTransformationMatrix::Request& req,
                                         phoxi_camera::SetTransformationMatrix::Response& res) {
        try {
            Eigen::Affine3d transform;
            tf::transformMsgToEigen(req.transform, transform);
            PhoXiInterface::setTransformation(transform.matrix(), req.coordinates_space, req.set_space,
                                              req.save_settings);
            //update dynamic reconfigure
            if (req.set_space) {
                dynamicReconfigureConfig.coordinate_space = req.coordinates_space;
            }
            dynamicReconfigureServer.updateConfig(dynamicReconfigureConfig);
            res.success = true;
            res.message = OKRESPONSE;
        } catch (PhoXiInterfaceException& e) {
            res.success = false;
            res.message = e.what();
        }
        return true;
    }

    bool
    RosInterface::saveLastFrame(phoxi_camera::SaveLastFrame::Request& req, phoxi_camera::SaveLastFrame::Response& res) {
        std::string file_path = req.file_path;

        try {
            // ~ error handling
            size_t pos = file_path.find('~');
            if (pos != std::string::npos) {
                char* home = std::getenv("HOME");
                if (!home) {
                    res.message = "'~' found in 'file_path' parameter but environment variable 'HOME' not found. Export' HOME' variable or pass absolute value to 'path' parameter.";
                    res.success = false;
                    return true;
                }
                file_path.replace(pos, 1, home);
            }

            // extension error handling
            const std::string extensions[] = {"praw", "ply", "ptx", "tif", "prawf"};
            bool correct_ext = false;
            pos = file_path.rfind('.') + 1;
            std::string extension = file_path.substr(pos, file_path.length());

            for (const auto& ext: extensions) {
                if (ext == extension) {
                    correct_ext = true;
                }
            }
            if (!correct_ext) {
                res.message = "Wrong extension.";
                res.success = false;
                return true;
            }

            // save last frame
            ROS_INFO("File path: %s", file_path.c_str());
            bool result = PhoXiInterface::saveLastFrame(file_path);
            if (result) {
                res.message = OKRESPONSE;
                res.success = true;
            } else {
                res.message = "Unsuccessful save.";
                res.success = false;
            }

        } catch (PhoXiInterfaceException& e) {
            res.success = false;
            res.message = e.what();
        }

        return true;
    }

#endif

    void RosInterface::getDefaultDynamicReconfigureConfig(phoxi_camera::phoxi_cameraConfig& config) {
        dynamicReconfigureServer.getConfigDefault(config);
        config.__fromServer__(nh);
    }

    bool RosInterface::fromPhoxiMeshToPolygonMesh(pho::api::PhoXiMesh& phoxiMesh, pcl::PolygonMesh& meshPcl) {
        try {
            pcl::PointCloud<pcl::PointXYZ> meshPointCloud{};

            for (auto point : phoxiMesh.Vertices) {
                meshPointCloud.push_back(pcl::PointXYZ(point.x, point.y, point.z));
            }
            pcl::toPCLPointCloud2(meshPointCloud, meshPcl.cloud);

            const uint triangleVertices = 3;
            uint numOfPolygons = phoxiMesh.Indices.size()/triangleVertices;
            meshPcl.polygons.resize(numOfPolygons);
            for (int i = 0; i < (numOfPolygons); i++) {
                for (int v = 0; v < triangleVertices; v++) {
                    meshPcl.polygons[i].vertices.push_back(phoxiMesh.Indices[i * triangleVertices + v]);
                }
            }
        } catch (PhoXiInterfaceException& e) {
            ROS_WARN("%s", e.what());
            return false;
        }
        return true;
    }

}







