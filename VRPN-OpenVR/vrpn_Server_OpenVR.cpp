#include <iostream>
#include <string>
#include <quat.h>
#include <vrpn_Connection.h>
#include "vrpn_Server_OpenVR.h"

vrpn_Server_OpenVR::vrpn_Server_OpenVR() {
    // Initialize OpenVR
    vr::EVRInitError eError = vr::VRInitError_None;
    vr = std::unique_ptr<vr::IVRSystem>(vr::VR_Init(&eError, vr::VRApplication_Utility/*VRApplication_Background*/)); /// https://github.com/ValveSoftware/openvr/wiki/API-Documentation
    if (eError != vr::VRInitError_None) {
//        vr.reset(nullptr);
        std::cerr << "Unable to init VR runtime: " << vr::VR_GetVRInitErrorAsEnglishDescription(eError) << std::endl;
    }
    else
        std::cerr << "VR_Init done" << std::endl;

    // Initialize VRPN Connection
    std::string connectionName = ":" + std::to_string(vrpn_DEFAULT_LISTEN_PORT_NO);
    connection = vrpn_create_server_connection(connectionName.c_str());
}


vrpn_Server_OpenVR::~vrpn_Server_OpenVR() {
    vr::VR_Shutdown();
    if (connection) {
        connection->removeReference();
        connection = NULL;
    }
}

void vrpn_Server_OpenVR::mainloop() {
    // Get Tracking Information
    vr::TrackedDevicePose_t m_rTrackedDevicePose[vr::k_unMaxTrackedDeviceCount];
    vr->GetDeviceToAbsoluteTrackingPose(    /// https://github.com/ValveSoftware/openvr/wiki/IVRSystem::GetDeviceToAbsoluteTrackingPose
        vr::TrackingUniverseStanding,
        0/*float fPredictedSecondsToPhotonsFromNow*/,
        m_rTrackedDevicePose,
        vr::k_unMaxTrackedDeviceCount
    );

    for (vr::TrackedDeviceIndex_t unTrackedDevice = 0; unTrackedDevice < vr::k_unMaxTrackedDeviceCount; unTrackedDevice++) {

        // Forget about disconnected devices
        if (!m_rTrackedDevicePose[unTrackedDevice].bDeviceIsConnected) {
            continue;
        }

        // get device class
        vr::ETrackedDeviceClass device_class_id = vr->GetTrackedDeviceClass(unTrackedDevice);
        const std::string device_class_name =
            vr::TrackedDeviceClass_HMD == device_class_id ? "HMD" :
            vr::TrackedDeviceClass_Controller == device_class_id ? "Controller" :
            vr::TrackedDeviceClass_GenericTracker == device_class_id ? "GenericTracker" :
            vr::TrackedDeviceClass_TrackingReference == device_class_id ? "TrackingReference" :
            vr::TrackedDeviceClass_DisplayRedirect == device_class_id ? "DisplayRedirect" :
            "Invalid";

        // find serial
        std::string device_serial = "";
        {
            /// https://steamcommunity.com/app/358720/discussions/0/1353742967802223832/
            unsigned int unRequiredBufferLen = 128;
            char* pchBuffer = new char[unRequiredBufferLen];
            unRequiredBufferLen = vr->GetStringTrackedDeviceProperty(unTrackedDevice, vr::Prop_SerialNumber_String, pchBuffer, unRequiredBufferLen, nullptr);
            device_serial = pchBuffer;
            delete[] pchBuffer;
        };

        // build name
        const std::string device_name = "openvr/" + device_class_name + "/" + (device_serial == "" ? std::to_string(unTrackedDevice) : device_serial);

        std::cerr << std::to_string(unTrackedDevice) << "=>" << " [" << device_name << "] "; /// https://github.com/ValveSoftware/openvr/wiki/IVRSystem_Overview

        // Treat different device types separately
        switch (device_class_id) {
            case vr::TrackedDeviceClass_HMD: {
                vrpn_Tracker_OpenVR_HMD *hmd{nullptr};
                auto search = hmds.find(unTrackedDevice);
                if (search == hmds.end()) {
                    std::unique_ptr<vrpn_Tracker_OpenVR_HMD> newHMD = std::make_unique<vrpn_Tracker_OpenVR_HMD>(device_name, connection, vr.get());
                    hmd = newHMD.get();
                    hmds[unTrackedDevice] = std::move(newHMD);
                } else {
                    hmd = search->second.get();
                }
                hmd->updateTracking(&m_rTrackedDevicePose[unTrackedDevice]);
                hmd->mainloop();
                break;
            }

            /*
                MUST READ:

                    # tracker is switching between TrackedDeviceClass_GenericTracker and TrackedDeviceClass_Controller
                    https://github.com/ValveSoftware/openvr/issues/902

                    # Recent Vive Tracker Changes
                    https://steamcommunity.com/games/250820/announcements/detail/1697186829260359619

            */
            case vr::TrackedDeviceClass_GenericTracker:     /// https://github.com/ValveSoftware/openvr/wiki/IVRSystem_Overview
            case vr::TrackedDeviceClass_TrackingReference:
            case vr::TrackedDeviceClass_Controller: {
                vrpn_Tracker_OpenVR_Controller *controller{nullptr};
                auto search = controllers.find(unTrackedDevice);
                if (search == controllers.end()) {
                    std::unique_ptr<vrpn_Tracker_OpenVR_Controller> newController = std::make_unique<vrpn_Tracker_OpenVR_Controller>(device_name, connection, vr.get());
                    controller = newController.get();
                    controllers[unTrackedDevice] = std::move(newController);
                } else {
                    controller = search->second.get();
                }
                controller->updateTracking(&m_rTrackedDevicePose[unTrackedDevice]);
                controller->updateController(unTrackedDevice);
                controller->mainloop();
                break;
            }

            default: {
                std::cerr << " NOT HANDLED ";
                break;
            }
        }

        std::cerr << std::endl;
    }

    // Send and receive all messages.
    connection->mainloop();

    // Bail if the connection is in trouble.
    if (!connection->doing_okay()) {
        std::cerr << "Connection is not doing ok. Should we bail?" << std::endl;
    }
}

