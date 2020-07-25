#include "vrpn_Tracker_Camera.h"
#include <openvr.h>
#include <quat.h>
#include <iostream>


vrpn_Tracker_Camera::vrpn_Tracker_Camera(const std::string& name, vrpn_Connection* connection, const std::string& tracker_serial, q_vec_type _arm) :
	vrpn_Tracker(name.c_str(), connection), name(name), tracker_serial(tracker_serial)
{
    arm[0] = _arm[0];
    arm[1] = _arm[1];
    arm[2] = _arm[2];

    // Initialize the vrpn_Tracker
    // We track each device separately so this will only ever have one sensor
    vrpn_Tracker::num_sensors = 1;
}

void vrpn_Tracker_Camera::updateTracking(q_vec_type& tracker_pos, q_type& tracker_quat, q_vec_type& reference_pos, q_type& reference_quat, q_vec_type& reference_point)
{
    pos[0] = tracker_pos[0];
    pos[1] = tracker_pos[1];
    pos[2] = tracker_pos[2];

    d_quat[0] = tracker_quat[0];
    d_quat[1] = tracker_quat[1];
    d_quat[2] = tracker_quat[2];
    d_quat[3] = tracker_quat[3];

    // Pack message
	vrpn_gettimeofday(&timestamp, NULL);
	char msgbuf[1000];
	vrpn_int32 len = vrpn_Tracker::encode_to(msgbuf);
	if (d_connection->pack_message(len, timestamp, position_m_id, d_sender_id, msgbuf, vrpn_CONNECTION_LOW_LATENCY)) {
		std::cerr << " Can't write message";
	}
}

void vrpn_Tracker_Camera::getRotation(q_type& q_current)
{
    q_current[0] = d_quat[0];
    q_current[1] = d_quat[1];
    q_current[2] = d_quat[2];
    q_current[3] = d_quat[3];
}

void vrpn_Tracker_Camera::getPosition(q_vec_type& vec)
{
    vec[0] = pos[0];
    vec[1] = pos[1];
    vec[2] = pos[2];
}

std::string vrpn_Tracker_Camera::getName()
{
    return name;
}

std::string vrpn_Tracker_Camera::getTrackerSerial()
{
    return tracker_serial;
}

void vrpn_Tracker_Camera::mainloop() {
    vrpn_gettimeofday( &(vrpn_Tracker_Camera::timestamp), NULL );
	vrpn_Tracker::server_mainloop();
}

