#include "vrpn_Tracker_OpenVR_HMD.h"

vrpn_Tracker_OpenVR_HMD::vrpn_Tracker_OpenVR_HMD(const std::string& name, vrpn_Connection* connection, vr::IVRSystem * vr, vr::ETrackedDeviceClass device_class_id) :
	vrpn_Tracker_OpenVR(name.c_str(), connection, vr, device_class_id)
{
}

void vrpn_Tracker_OpenVR_HMD::mainloop() {
	vrpn_Tracker_OpenVR::mainloop();
}