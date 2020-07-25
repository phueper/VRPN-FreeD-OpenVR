#include <iostream>
#include <string>
#include <quat.h>
#include <vrpn_Connection.h>
#include "vrpn_Server_OpenVR.h"
#include "console.h"

vrpn_Server_OpenVR::vrpn_Server_OpenVR() {
    // Initialize OpenVR
    vr::EVRInitError eError = vr::VRInitError_None;
    vr = std::unique_ptr<vr::IVRSystem>(vr::VR_Init(&eError, vr::VRApplication_Utility/*VRApplication_Background*/)); /// https://github.com/ValveSoftware/openvr/wiki/API-Documentation
    if (eError != vr::VRInitError_None)
    {
//        vr.reset(nullptr);
        std::cerr << "Unable to init VR runtime: " << vr::VR_GetVRInitErrorAsEnglishDescription(eError) << std::endl;
        exit(1);
    };

    // Initialize VRPN Connection
    std::string connectionName = ":" + std::to_string(vrpn_DEFAULT_LISTEN_PORT_NO);
    connection = vrpn_create_server_connection(connectionName.c_str());

    console_setup(&console_in, &console_out);
}


vrpn_Server_OpenVR::~vrpn_Server_OpenVR() {
    vr::VR_Shutdown();
    if (connection) {
        connection->removeReference();
        connection = NULL;
    }
}

void vrpn_Server_OpenVR::mainloop() {
    char* buf = NULL;

    // Get Tracking Information
    vr::TrackedDevicePose_t m_rTrackedDevicePose[vr::k_unMaxTrackedDeviceCount];
    vr->GetDeviceToAbsoluteTrackingPose(    /// https://github.com/ValveSoftware/openvr/wiki/IVRSystem::GetDeviceToAbsoluteTrackingPose
        vr::TrackingUniverseStanding,
        0/*float fPredictedSecondsToPhotonsFromNow*/,
        m_rTrackedDevicePose,
        vr::k_unMaxTrackedDeviceCount
    );

    // setup cusrsor to top
    SetConsoleCursorPosition(console_out, { 0, 0 });

    buf = NULL;
    asprintf(&buf, "VRPN/FREE-D for StreamVR. api %s, app built [" __DATE__ " " __TIME__ "]", vr->GetRuntimeVersion());
    console_put(buf);
    if (buf) free(buf);
    console_put("");

    for (vr::TrackedDeviceIndex_t unTrackedDevice = 0; unTrackedDevice < vr::k_unMaxTrackedDeviceCount; unTrackedDevice++) {
        const char* state = "Running_OK";
        int f_update_data = 1;
        vr::TrackedDevicePose_t* pose = &m_rTrackedDevicePose[unTrackedDevice];

        if (!pose->bDeviceIsConnected)
            continue;

        if (!pose->bPoseIsValid) {
            state = " !bPoseIsValid";
            f_update_data = 0;
        }

        if (pose->eTrackingResult != vr::TrackingResult_Running_OK)
        {
            state =
                vr::TrackingResult_Uninitialized == pose->eTrackingResult ? "Uninitialized" :
                vr::TrackingResult_Calibrating_InProgress == pose->eTrackingResult ? "Calibrating In Progress" :
                vr::TrackingResult_Calibrating_OutOfRange == pose->eTrackingResult ? "Calibrating Out Of Range" :
                vr::TrackingResult_Running_OutOfRange == pose->eTrackingResult ? "Running Out Of Range" :
                "Unknown";
            f_update_data = 0;
        }


        // get device class
        vr::ETrackedDeviceClass device_class_id = vr->GetTrackedDeviceClass(unTrackedDevice);
        const std::string device_class_name = getDeviceClassName(device_class_id);

        // find serial
        std::string device_serial = getDeviceSerial(unTrackedDevice, vr.get());

        // build name
        const std::string device_name = "openvr/" + device_class_name + "/" + (device_serial == "" ? std::to_string(unTrackedDevice) : device_serial);

        /* output name */
        buf = NULL;  asprintf(&buf, "[%2d] => %-40s | %-40s", unTrackedDevice, device_name.c_str(), state);
        console_put(buf);
        if (!buf) free(buf);

        /* create new device or get early added */
        vrpn_Tracker_OpenVR *dev{ nullptr };
        auto dev_srch = devices.find(unTrackedDevice);
        if (dev_srch == devices.end())
        {
            std::unique_ptr<vrpn_Tracker_OpenVR> newDEV;

            switch (device_class_id)
            {
                case vr::TrackedDeviceClass_GenericTracker:     /// https://github.com/ValveSoftware/openvr/wiki/IVRSystem_Overview
                case vr::TrackedDeviceClass_TrackingReference:
                case vr::TrackedDeviceClass_HMD:
                    newDEV = std::make_unique<vrpn_Tracker_OpenVR_HMD>(device_name, connection, vr.get(), unTrackedDevice);
                    break;

                case vr::TrackedDeviceClass_Controller:
                    newDEV = std::make_unique<vrpn_Tracker_OpenVR_Controller>(device_name, connection, vr.get(), unTrackedDevice);
                    break;

                default:
                    newDEV = std::make_unique<vrpn_Tracker_OpenVR>(device_name, connection, vr.get(), unTrackedDevice);
            }

            dev = newDEV.get();
            devices[unTrackedDevice] = std::move(newDEV);
        }
        else
            dev = dev_srch->second.get();

        /* update tracking data */
        if (f_update_data)
        {
            dev->updateTracking(pose);
            dev->mainloop();
        };

        /* display position and rot */
        q_vec_type vec;
        dev->getPosition(vec);
        q_type quat;
        dev->getRotation(quat);
        q_vec_type yawPitchRoll;
        q_to_euler(yawPitchRoll, quat); // quaternion to euler for display
        /*
            resulting vector is:

            [0] - Q_YAW - rotation about Z
            [1] - Q_PITCH - rotation about Y
            [2] - Q_ROLL - rotation about X
        */
        buf = NULL; asprintf(&buf, "        pos=[%8.4f, %8.4f, %8.4f], euler=[%8.4f, %8.4f, %8.4f]",
            vec[0], vec[1], vec[2],
            yawPitchRoll[2] * 180.0 / 3.1415926,
            yawPitchRoll[1] * 180.0 / 3.1415926,
            yawPitchRoll[0] * 180.0 / 3.1415926);
        console_put(buf);
        if (!buf) free(buf);

        /* empty line */
        console_put("");
    }

    // Send and receive all messages.
    connection->mainloop();

#if 0
    // iterate controllers
    for (vr::TrackedDeviceIndex_t unTrackedDevice = 0; unTrackedDevice < vr::k_unMaxTrackedDeviceCount; unTrackedDevice++)
    {
        auto search = controllers.find(unTrackedDevice);
        if (search == controllers.end())
            continue;
        vrpn_Tracker_OpenVR_Controller *controller = search->second.get();
        std::cerr << "known:" << controller->getName() << std::endl;
        
    }
#endif

    // Bail if the connection is in trouble.
    if (!connection->doing_okay()) {
        std::cerr << "Connection is not doing ok. Should we bail?" << std::endl;
    }
}

const std::string vrpn_Server_OpenVR::getDeviceClassName(vr::ETrackedDeviceClass device_class_id)
{
    return
        vr::TrackedDeviceClass_HMD == device_class_id ? "HMD" :
        vr::TrackedDeviceClass_Controller == device_class_id ? "Controller" :
        vr::TrackedDeviceClass_GenericTracker == device_class_id ? "GenericTracker" :
        vr::TrackedDeviceClass_TrackingReference == device_class_id ? "TrackingReference" :
        vr::TrackedDeviceClass_DisplayRedirect == device_class_id ? "DisplayRedirect" :
        "Invalid";
}

const std::string vrpn_Server_OpenVR::getDeviceSerial(vr::TrackedDeviceIndex_t trackedDeviceIndex, vr::IVRSystem * vr)
{
    std::string device_serial = "";
    {
        /// https://steamcommunity.com/app/358720/discussions/0/1353742967802223832/
        unsigned int unRequiredBufferLen = 128;
        char* pchBuffer = new char[unRequiredBufferLen];
        unRequiredBufferLen = vr->GetStringTrackedDeviceProperty(trackedDeviceIndex, vr::Prop_SerialNumber_String, pchBuffer, unRequiredBufferLen, nullptr);
        device_serial = pchBuffer;
        delete[] pchBuffer;
    };
    return device_serial;
}

