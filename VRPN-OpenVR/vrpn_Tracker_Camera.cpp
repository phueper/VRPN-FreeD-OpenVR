#include "vrpn_Tracker_Camera.h"
#include <openvr.h>
#include <quat.h>
#include <iostream>
#include "FreeD.h"

void vrpn_Tracker_Camera::filterAdd(filter_abstract* flt)
{
    filters_list[filters_cnt] = flt;
    filters_cnt++;
};

vrpn_Tracker_Camera::vrpn_Tracker_Camera(int idx, const std::string& name, vrpn_Connection* connection, const std::string& tracker_serial, q_vec_type _arm) :
	vrpn_Tracker(name.c_str(), connection), name(name), tracker_serial(tracker_serial), freed_socket(-1), idx(idx)
{
    arm[0] = _arm[0];
    arm[1] = _arm[1];
    arm[2] = _arm[2];

    filters_cnt = 0;

    // Initialize the vrpn_Tracker
    // We track each device separately so this will only ever have one sensor
    vrpn_Tracker::num_sensors = 1;
}

/*
    OpenVR world:

        right-handed system
        +y is up
        +x is to the right
        -z is forward
        Distance unit is  meters

    UE4 world:

        Unreal uses a right-handed, Z-up coordinate system.
        +x - forward
        +y - right
        +z - up

*/
static inline void quat_openvr_to_ue4(q_type src, q_type& dst)
{
#if 0 // lets do preroate early
    q_type prerot90, tmp;

    q_from_euler(prerot90, 0, 0, -3.1415926 / 2.0); // double yaw, double pitch, double roll
    q_mult(tmp, src, prerot90);

    dst[0] = -tmp[2];
    dst[1] = tmp[0];
    dst[2] = tmp[1];
    dst[3] = -tmp[3];
#else
    dst[0] = -src[2];
    dst[1] = src[0];
    dst[2] = src[1];
    dst[3] = src[3];
#endif
}

static inline void vec_openvr_to_ue4(q_vec_type src, q_vec_type& dst)
{
    dst[0] = -src[2];
    dst[1] = src[0];
    dst[2] = src[1];
}

void vrpn_Tracker_Camera::updateTracking(q_vec_type _tracker_pos, q_type _tracker_quat, q_vec_type reference_pos, q_type reference_quat, q_vec_type reference_point, struct timeval *tv)
{
    // backup origin data sent to update tracking
    q_vec_type tracker_pos;
    q_type tracker_quat;
    q_vec_copy(tracker_pos, _tracker_pos);
    q_copy(tracker_quat, _tracker_quat);

    // apply filters
    int f;
    q_vec_type tmp_tracker_pos;
    q_type tmp_tracker_quat;
    q_vec_copy(tmp_tracker_pos, tracker_pos);
    q_copy(tmp_tracker_quat, tracker_quat);
    for (f = 0; f < filters_cnt; f++)
        filters_list[f]->process_data(tmp_tracker_pos, tmp_tracker_quat);
    q_vec_copy(tracker_pos, tmp_tracker_pos);
    q_copy(tracker_quat, tmp_tracker_quat);

    // -------------------------------------------------------

    q_type ue4_tracker_quat, ue4_reference_quat;

    // translate all quats to UE4
    quat_openvr_to_ue4(tracker_quat, ue4_tracker_quat);
    quat_openvr_to_ue4(reference_quat, ue4_reference_quat);

    // cam re-rotation
    q_type i_ue4_reference_quat;
    q_invert(i_ue4_reference_quat, ue4_reference_quat);
    q_mult(d_quat, ue4_tracker_quat, i_ue4_reference_quat);

    // -------------------------------------------------------

    q_vec_type ue4_tracker_pos, ue4_reference_pos;

    // translate all pos to UE4

    vec_openvr_to_ue4(tracker_pos, ue4_tracker_pos);
    vec_openvr_to_ue4(reference_pos, ue4_reference_pos);

    // find relative vector of movement
    q_vec_subtract(pos, ue4_tracker_pos, ue4_reference_pos);

    // get the initial rotation of refernce quat
    q_vec_type yawPitchRoll;
    q_to_euler(yawPitchRoll, ue4_reference_quat);

    // do a rotation of it
    q_type rot_z;
    q_from_euler(rot_z, yawPitchRoll[0], 0, 0);
    q_xform(pos, rot_z, pos);

    // add reference point
    q_vec_add(pos, pos, reference_point);

    // add arm
    q_vec_type arm_vec;
    q_type arm_quat;
    q_invert(arm_quat, d_quat);
    q_xform(arm_vec, arm_quat, arm);
    q_vec_add(pos, pos, arm_vec);

    // Pack message
#if 0
	vrpn_gettimeofday(&timestamp, NULL);
#else
    timestamp.tv_sec = tv->tv_sec;
    timestamp.tv_usec = tv->tv_usec;
#endif
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
//    vrpn_gettimeofday( &(vrpn_Tracker_Camera::timestamp), NULL );
	vrpn_Tracker::server_mainloop();
    freedSend();
}

void vrpn_Tracker_Camera::freedAdd(char *host_port)
{
    char *port, *host = strdup(host_port);

    port = strrchr(host, ':');
    if (port)
    {
        struct sockaddr_in addr;

        *port = 0; port++;

        /* prepare address */
        addr.sin_family = AF_INET;
        addr.sin_addr.s_addr = inet_addr(host);
        addr.sin_port = htons((unsigned short)atoi(port));

        /* store address */
        freed_sockaddr.push_back(addr);
    }

    free(host);
}

void vrpn_Tracker_Camera::freedSend()
{
    FreeD_D1_t freed;
    unsigned char buf[FREE_D_D1_PACKET_SIZE];

    /* init socket */
    if (freed_socket <= 0)
        freed_socket = socket(AF_INET, SOCK_DGRAM, 0);

    if (freed_socket <= 0)
        return;

    memset(&freed, 0, sizeof(freed));

    freed.ID = idx + 1;

    q_vec_type pos;
    getPosition(pos);
    freed.X = pos[0] * 1000.0;
    freed.Y = pos[1] * 1000.0;
    freed.Z = pos[2] * 1000.0;

    q_type quat;
    getRotation(quat);
    q_vec_type yawPitchRoll;
    q_to_euler(yawPitchRoll, quat);
    freed.Pan = yawPitchRoll[0] * 180.0 / 3.1415926;
    freed.Roll = yawPitchRoll[2] * 180.0 / 3.1415926;
    freed.Tilt = yawPitchRoll[1] * 180.0 / 3.1415926;

    FreeD_D1_pack(buf, FREE_D_D1_PACKET_SIZE, &freed);

    for (const struct sockaddr_in /*auto&*/ addr : freed_sockaddr)
    {
        sendto
        (
            freed_socket,               /* Socket to send result */
            (char*)buf,                 /* The datagram buffer */
            sizeof(buf),                /* The datagram lngth */
            0,                          /* Flags: no options */
            (struct sockaddr *)&addr,   /* addr */
            sizeof(struct sockaddr_in)  /* Server address length */
        );
    }
}

