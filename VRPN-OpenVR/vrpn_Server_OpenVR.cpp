#include <iostream>
#include <string>
#include <quat.h>
#include <vrpn_Connection.h>
#include "vrpn_Server_OpenVR.h"
#include "console.h"

vrpn_Server_OpenVR::vrpn_Server_OpenVR(int argc, char *argv[])
{
    std::string connectionName = "";
    int listen_vrpn_port = vrpn_DEFAULT_LISTEN_PORT_NO;

    // Initialize OpenVR
    vr::EVRInitError eError = vr::VRInitError_None;
    vr = std::unique_ptr<vr::IVRSystem>(vr::VR_Init(&eError, vr::VRApplication_Utility/*VRApplication_Background*/)); /// https://github.com/ValveSoftware/openvr/wiki/API-Documentation
    if (eError != vr::VRInitError_None)
    {
//        vr.reset(nullptr);
        std::cerr << "Unable to init VR runtime: " << vr::VR_GetVRInitErrorAsEnglishDescription(eError) << std::endl;
        exit(1);
    };

    // Process arguments
    if (argc > 1)
    {
        for (int p = 1; p < argc;)
        {
            if (!strcmp(argv[p], "port") && (p + 1) <= argc)        // 1 argument: port <listen port>
            {
                listen_vrpn_port = atoi(argv[p + 1]);
                p += 2;
            }
            else if (!strcmp(argv[p], "ref") && (p + 3) <= argc)    // 3 argument: ref <x> <y> <z>
            {
                reference_point[0] = atof(argv[p + 1]);
                reference_point[1] = atof(argv[p + 2]);
                reference_point[2] = atof(argv[p + 3]);
                p += 4;
            }
            else if (!strcmp(argv[p], "cam") && (p + 5) <= argc)    // 5 argument: cam <NAME> <TRACKER SERIAL> <x> <y> <z>
            {
                // Initialize VRPN Connection
                if (connectionName == "")
                {
                    connectionName = ":" + std::to_string(listen_vrpn_port);
                    connection = vrpn_create_server_connection(connectionName.c_str());
                }

                q_vec_type arm;
                std::unique_ptr<vrpn_Tracker_Camera> newCAM;
                std::string name, serial;

                // build name
                name = "virtual/"; name += argv[p + 1];

                // serial
                serial = argv[p + 2];

                // build arm
                arm[0] = atof(argv[p + 3]);
                arm[1] = atof(argv[p + 4]);
                arm[2] = atof(argv[p + 5]);

                // build cam class
                newCAM = std::make_unique<vrpn_Tracker_Camera>(name, connection, serial, arm);
                cameras.push_back(std::move(newCAM));

                p += 6;
            }
            else
            {
                std::cerr << "Failed to parse argument [" << argv[p] << "], either unknown or wrong parameters count" << std::endl;
                exit(1);
            }
        }
    }

    // Initialize VRPN Connection
    if (connectionName == "")
    {
        connectionName = ":" + std::to_string(listen_vrpn_port);
        connection = vrpn_create_server_connection(connectionName.c_str());
    }

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
    char *buf = NULL, press;
    int ref_tracker_idx = -1;

    press = console_keypress(console_in);
    if (press >= '0' && press <= '9')
        ref_tracker_idx = press - '0';

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
//    console_cls(GetStdHandle(STD_ERROR_HANDLE));
//    console_cls(GetStdHandle(STD_OUTPUT_HANDLE));

    // show built info
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
        buf = NULL; asprintf(&buf, "        pos=[%8.4f, %8.4f, %8.4f], euler=[Yaw/Z=%8.4f, Pitch/Y=%8.4f, Roll/X=%8.4f]",
            vec[0], vec[1], vec[2],
            yawPitchRoll[0] * 180.0 / 3.1415926,
            yawPitchRoll[1] * 180.0 / 3.1415926,
            yawPitchRoll[2] * 180.0 / 3.1415926);
        console_put(buf);
        if (!buf) free(buf);

        /* find camera assiciated with that tracker */
        for (const auto& ci : cameras)
        {
            /* check for serial */
            if (ci->getTrackerSerial() != device_serial)
                continue;

            /* do some precomputation */
            ci->updateTracking(vec, quat, reference_position, reference_quat, reference_point);
        }

        /* empty line */
        console_put("");

        /* save tracker data as reference position */
        if (ref_tracker_idx == unTrackedDevice)
        {
            dev->getPosition(reference_position);
            dev->getRotation(reference_quat);
        }
    }

    console_put("Virtual space:");
    console_put("");

    {
        /* display reference point */
        buf = NULL; asprintf(&buf, "        ref=[%8.4f, %8.4f, %8.4f]",
            reference_point[0], reference_point[1], reference_point[2]);
        console_put(buf);
        if (!buf) free(buf);

        /* display reference position and rot */
        q_vec_type vec;
        vec[0] = reference_position[0]; vec[1] = reference_position[1]; vec[2] = reference_position[2];
        q_type quat;
        quat[0] = reference_quat[0]; quat[1] = reference_quat[1]; quat[2] = reference_quat[2]; quat[3] = reference_quat[3];
        q_vec_type yawPitchRoll;
        q_to_euler(yawPitchRoll, quat); // quaternion to euler for display
        /*
            resulting vector is:

            [0] - Q_YAW - rotation about Z
            [1] - Q_PITCH - rotation about Y
            [2] - Q_ROLL - rotation about X
        */
        buf = NULL; asprintf(&buf, "        pos=[%8.4f, %8.4f, %8.4f], euler=[Yaw/Z=%8.4f, Pitch/Y=%8.4f, Roll/X=%8.4f]",
            vec[0], vec[1], vec[2],
            yawPitchRoll[0] * 180.0 / 3.1415926,
            yawPitchRoll[1] * 180.0 / 3.1415926,
            yawPitchRoll[2] * 180.0 / 3.1415926);
        console_put(buf);
        if (!buf) free(buf);

        /* empty line */
        console_put("");
    }
#if 1
    console_put("Virtual cameras:");
    console_put("");

    /* dump all cameras state */
    for (const auto& ci : cameras)
    {
        /* output name */
        buf = NULL;  asprintf(&buf, "        %-40s | %-40s", ci->getName().c_str(), ci->getTrackerSerial().c_str());
        console_put(buf);
        if (!buf) free(buf);

        /* display position and rot */
        q_vec_type vec;
        ci->getPosition(vec);
        q_type quat;
        ci->getRotation(quat);
        q_vec_type yawPitchRoll;
        q_to_euler(yawPitchRoll, quat); // quaternion to euler for display
        /*
            resulting vector is:

            [0] - Q_YAW - rotation about Z
            [1] - Q_PITCH - rotation about Y
            [2] - Q_ROLL - rotation about X
        */
        buf = NULL; asprintf(&buf, "        pos=[%8.4f, %8.4f, %8.4f], euler=[Yaw/Z=%8.4f, Pitch/Y=%8.4f, Roll/X=%8.4f]",
            vec[0], vec[1], vec[2],
            yawPitchRoll[0] * 180.0 / 3.1415926,
            yawPitchRoll[1] * 180.0 / 3.1415926,
            yawPitchRoll[2] * 180.0 / 3.1415926);
        console_put(buf);
        if (!buf) free(buf);

        /* empty line */
        console_put("");
    }

    /* empty line */
    console_put("");
#endif
    // Send and receive all messages.
    connection->mainloop();

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

