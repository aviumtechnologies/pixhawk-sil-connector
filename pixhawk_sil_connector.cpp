/*  File    : pixhawk_sil_connector.cpp
 *  Abstract:
 *
 *  Simulink C++ S-function for software-in-the-loop (SIL) simulation with Pixhawk.
 *
 *  Copyright (c) 2022 Kiril Boychev
 */

#include <algorithm>
#include <iostream>
#include <iterator>
#include <sstream>
#include <string>

#include <asio.hpp>
#include "mavlink/common/mavlink.h"

#define S_FUNCTION_LEVEL 2
#define S_FUNCTION_NAME pixhawk_sil_connector

static asio::io_service io_service;
static asio::ip::tcp::socket sock(io_service);
static std::string eStatus;
static mavlink_hil_actuator_controls_t hil_actuator_controls_msg;

/*
 * Need to include simstruc.h for the definition of the SimStruct and
 * its associated macro definitions.
 */
#include "simstruc.h"

/*====================*
 * S-function methods *
 *====================*/

/* Function: mdlInitializeSizes ===============================================
 * Abstract:
 *    The sizes information is used by Simulink to determine the S-function
 *    block's characteristics (number of inputs, outputs, states, etc.).
 */
static void mdlInitializeSizes(SimStruct *S)
{
    ssSetNumSFcnParams(S, 0); /* Number of expected parameters */

    if (ssGetNumSFcnParams(S) != ssGetSFcnParamsCount(S))
    {
        return;
    }

    if (!ssSetNumInputPorts(S, 11)) /* Number of input ports */
    {
        return;
    }

    ssSetInputPortWidth(S, 0, 1); // time (s)
    ssSetInputPortDirectFeedThrough(S, 0, 1);
    ssSetInputPortWidth(S, 1, 3); // A_measured (m/s^2)
    ssSetInputPortDirectFeedThrough(S, 1, 1);
    ssSetInputPortWidth(S, 2, 3); // omega_b_measured (rad/s)
    ssSetInputPortDirectFeedThrough(S, 2, 1);
    ssSetInputPortWidth(S, 3, 3); // B_measured (gauss)
    ssSetInputPortDirectFeedThrough(S, 3, 1);
    ssSetInputPortWidth(S, 4, 1); // P_measured (hPa)
    ssSetInputPortDirectFeedThrough(S, 4, 1);
    ssSetInputPortWidth(S, 5, 1); // T_measured (degC)
    ssSetInputPortDirectFeedThrough(S, 5, 1);
    ssSetInputPortWidth(S, 6, 1); // h_measured (m)
    ssSetInputPortDirectFeedThrough(S, 6, 1);
    ssSetInputPortWidth(S, 7, 1); // q_measured (hPa)
    ssSetInputPortDirectFeedThrough(S, 7, 1);
    ssSetInputPortWidth(S, 8, 12); // xyz_measured (-)
    ssSetInputPortDirectFeedThrough(S, 8, 1);
    ssSetInputPortWidth(S, 9, 13); // rc_channels (-)
    ssSetInputPortDirectFeedThrough(S, 9, 1);
    ssSetInputPortWidth(S, 10, 17); // ground_truth (-)
    ssSetInputPortDirectFeedThrough(S, 10, 1);

    if (!ssSetNumOutputPorts(S, 1)) /* Number of output ports */
    {
        return;
    }

    /* Output ports size */
    ssSetOutputPortWidth(S, 0, 16);

    /* Number of sample times */
    ssSetNumSampleTimes(S, 1);

    /* Number of PWork vector */
    ssSetNumPWork(S, 2);

    ssSetOptions(S, 0);
}

/* Function: mdlInitializeSampleTimes =========================================
 * Abstract:
 *    This function is used to specify the sample time(s) for your
 *    S-function. You must register the same number of sample times as
 *    specified in ssSetNumSampleTimes.
 */
static void mdlInitializeSampleTimes(SimStruct *S)
{
    ssSetSampleTime(S, 0, 0.004);
    ssSetOffsetTime(S, 0, 0.0);
    ssSetModelReferenceSampleTimeDefaultInheritance(S);
}

#define MDL_START /* Change to #undef to remove function */
#if defined(MDL_START)
/* Function: mdlStart =======================================================
 * Abstract:
 *    This function is called once at start of model execution. If you
 *    have states that should be initialized once, this is the place
 *    to do it.
 */
static void mdlStart(SimStruct *S)
{
    try
    {

        asio::ip::tcp::endpoint endpoint(asio::ip::address::from_string("0.0.0.0"), 4560);
        asio::ip::tcp::acceptor acceptor(io_service, endpoint);

        mexPrintf("Waiting for PX4 to connect on TCP port 4560...\n");

        acceptor.accept(sock);

        mexPrintf("PX4 connected on TCP port 4560.\n");

        ssSetPWorkValue(S, 0, (void *)&sock);

        uint8_t *buffer = nullptr;
        buffer = (uint8_t *)calloc(1024, 1);
        ssSetPWorkValue(S, 1, (void *)buffer);
    }
    catch (const std::exception &e)
    {
        eStatus = std::string(e.what());
        ssSetErrorStatus(S, eStatus.c_str());
    }
}
#endif /*  MDL_START */

/* Function: mdlOutputs =======================================================
 * Abstract:
 *    In this function, you compute the outputs of your S-function
 *    block.
 */
static void mdlOutputs(SimStruct *S, int_T tid)
{
    try
    {
        asio::ip::tcp::socket *sock = (asio::ip::tcp::socket *)(ssGetPWorkValue(S, 0));

        uint8_t *buffer = (uint8_t *)ssGetPWorkValue(S, 1);

        if (!sock)
        {
            return;
        }

        InputRealPtrsType time = ssGetInputPortRealSignalPtrs(S, 0);
        InputRealPtrsType A_measured = ssGetInputPortRealSignalPtrs(S, 1);
        InputRealPtrsType omega_b_measured = ssGetInputPortRealSignalPtrs(S, 2);
        InputRealPtrsType B_measured = ssGetInputPortRealSignalPtrs(S, 3);
        InputRealPtrsType P_measured = ssGetInputPortRealSignalPtrs(S, 4);
        InputRealPtrsType T_measured = ssGetInputPortRealSignalPtrs(S, 5);
        InputRealPtrsType h_measured = ssGetInputPortRealSignalPtrs(S, 6);
        InputRealPtrsType q_measured = ssGetInputPortRealSignalPtrs(S, 7);
        InputRealPtrsType xyz_measured = ssGetInputPortRealSignalPtrs(S, 8);
        InputRealPtrsType rc_channels = ssGetInputPortRealSignalPtrs(S, 9);
        InputRealPtrsType ground_truth = ssGetInputPortRealSignalPtrs(S, 10);

        memset(buffer, 0, 1024);

        mavlink_message_t encoded_msg;

        mavlink_heartbeat_t heartbeat_msg;
        heartbeat_msg.autopilot = (uint8_t)MAV_AUTOPILOT_GENERIC;
        heartbeat_msg.type = (uint8_t)MAV_TYPE_GENERIC;
        heartbeat_msg.system_status=(uint8_t)0;
        heartbeat_msg.base_mode=(uint8_t)0;
        heartbeat_msg.custom_mode=(uint32_t)0;

        mavlink_msg_heartbeat_encode_chan(1, 200, MAVLINK_COMM_0, &encoded_msg, &heartbeat_msg);

        auto bytesToSend = mavlink_msg_to_send_buffer(&buffer[0], &encoded_msg);

        mavlink_hil_sensor_t hil_sensor_msg;
        hil_sensor_msg.time_usec = (uint64_t)((*time[0]) * 1e6);
        hil_sensor_msg.xacc = (float)(*A_measured[0]);
        hil_sensor_msg.yacc = (float)(*A_measured[1]);
        hil_sensor_msg.zacc = (float)(*A_measured[2]);
        hil_sensor_msg.xgyro = (float)(*omega_b_measured[0]);
        hil_sensor_msg.ygyro = (float)(*omega_b_measured[1]);
        hil_sensor_msg.zgyro = (float)(*omega_b_measured[2]);
        hil_sensor_msg.xmag = (float)(*B_measured[0]);
        hil_sensor_msg.ymag = (float)(*B_measured[1]);
        hil_sensor_msg.zmag = (float)(*B_measured[2]);
        hil_sensor_msg.abs_pressure = (float)(*P_measured[0]);
        hil_sensor_msg.diff_pressure = (float)(*q_measured[0]);
        hil_sensor_msg.temperature = (float)(*T_measured[0]);
        hil_sensor_msg.fields_updated = (uint32_t)0x1FFF;
        hil_sensor_msg.id = (uint8_t)0;

        mavlink_msg_hil_sensor_encode_chan(1, 200, MAVLINK_COMM_0, &encoded_msg, &hil_sensor_msg);

        bytesToSend += mavlink_msg_to_send_buffer(&buffer[bytesToSend], &encoded_msg);

        mavlink_hil_gps_t hil_gps_msg;
        hil_gps_msg.time_usec = (uint64_t)((*time[0]) * 1e6);
        hil_gps_msg.fix_type = (uint8_t)(*xyz_measured[0]);
        hil_gps_msg.lat = (int32_t)(*xyz_measured[1]);
        hil_gps_msg.lon = (int32_t)(*xyz_measured[2]);
        hil_gps_msg.alt = (int32_t)(*xyz_measured[3]);
        hil_gps_msg.eph = (uint16_t)(*xyz_measured[4]);
        hil_gps_msg.epv = (uint16_t)(*xyz_measured[5]);
        hil_gps_msg.vel = (uint16_t)std::floor(*xyz_measured[6]);
        hil_gps_msg.vn = (int16_t)std::floor(*xyz_measured[7]);
        hil_gps_msg.ve = (int16_t)std::floor(*xyz_measured[8]);
        hil_gps_msg.vd = (int16_t)std::floor(*xyz_measured[9]);
        hil_gps_msg.cog = (uint16_t)(*xyz_measured[10]);
        hil_gps_msg.satellites_visible = (uint8_t)(*xyz_measured[11]);
        hil_gps_msg.id = (uint8_t)0;
        hil_gps_msg.yaw = (uint16_t)0;

        mavlink_msg_hil_gps_encode_chan(1, 200, MAVLINK_COMM_0, &encoded_msg, &hil_gps_msg);

        bytesToSend += mavlink_msg_to_send_buffer(&buffer[bytesToSend], &encoded_msg);

        mavlink_hil_rc_inputs_raw_t hil_rc_inputs_raw;
        hil_rc_inputs_raw.time_usec = (uint64_t)((*time[0]) * 1e6);
        hil_rc_inputs_raw.chan1_raw = (uint16_t)(*rc_channels[0]);
        hil_rc_inputs_raw.chan2_raw = (uint16_t)(*rc_channels[1]);
        hil_rc_inputs_raw.chan3_raw = (uint16_t)(*rc_channels[2]);
        hil_rc_inputs_raw.chan4_raw = (uint16_t)(*rc_channels[3]);
        hil_rc_inputs_raw.chan5_raw = (uint16_t)(*rc_channels[4]);
        hil_rc_inputs_raw.chan6_raw = (uint16_t)(*rc_channels[5]);
        hil_rc_inputs_raw.chan7_raw = (uint16_t)(*rc_channels[6]);
        hil_rc_inputs_raw.chan8_raw = (uint16_t)(*rc_channels[7]);
        hil_rc_inputs_raw.chan9_raw = (uint16_t)(*rc_channels[8]);
        hil_rc_inputs_raw.chan10_raw = (uint16_t)(*rc_channels[9]);
        hil_rc_inputs_raw.chan11_raw = (uint16_t)(*rc_channels[10]);
        hil_rc_inputs_raw.chan12_raw = (uint16_t)(*rc_channels[11]);
        hil_rc_inputs_raw.rssi = (uint8_t)(*rc_channels[12]);

        mavlink_msg_hil_rc_inputs_raw_encode_chan(1, 200, MAVLINK_COMM_0, &encoded_msg, &hil_rc_inputs_raw);

        bytesToSend += mavlink_msg_to_send_buffer(&buffer[bytesToSend], &encoded_msg);

        float phi = (float)(*ground_truth[0]);
        float theta = (float)(*ground_truth[1]);
        float psi = (float)(*ground_truth[2]);

        float sphi = std::sin(phi / 2.0);
        float stheta = std::sin(theta / 2.0);
        float spsi = std::sin(psi / 2.0);
        float cphi = std::cos(phi / 2.0);
        float ctheta = std::cos(theta / 2.0);
        float cpsi = std::cos(psi / 2.0);

        float attitude_quaternion[4];
        attitude_quaternion[0] = cpsi * ctheta * cphi + spsi * stheta * sphi;
        attitude_quaternion[1] = cpsi * ctheta * sphi - spsi * stheta * cphi;
        attitude_quaternion[2] = cpsi * stheta * cphi + spsi * ctheta * sphi;
        attitude_quaternion[3] = spsi * ctheta * cphi - cpsi * stheta * sphi;

        mavlink_hil_state_quaternion_t hil_state_quaternion;
        hil_state_quaternion.attitude_quaternion[0] = attitude_quaternion[0];
        hil_state_quaternion.attitude_quaternion[1] = attitude_quaternion[1];
        hil_state_quaternion.attitude_quaternion[2] = attitude_quaternion[2];
        hil_state_quaternion.attitude_quaternion[3] = attitude_quaternion[3];
        hil_state_quaternion.rollspeed = (float)(*ground_truth[3]);
        hil_state_quaternion.pitchspeed = (float)(*ground_truth[4]);
        hil_state_quaternion.yawspeed = (float)(*ground_truth[5]);
        hil_state_quaternion.lat = (int32_t)(*ground_truth[6]);
        hil_state_quaternion.lon = (int32_t)(*ground_truth[7]);
        hil_state_quaternion.alt = (int32_t)(*ground_truth[8]);
        hil_state_quaternion.vx = (int16_t)(*ground_truth[9]);
        hil_state_quaternion.vy = (int16_t)(*ground_truth[10]);
        hil_state_quaternion.vz = (int16_t)(*ground_truth[11]);
        hil_state_quaternion.ind_airspeed = (uint16_t)(*ground_truth[12]);
        hil_state_quaternion.true_airspeed = (uint16_t)(*ground_truth[13]);
        hil_state_quaternion.xacc = (int16_t)(*ground_truth[14]);
        hil_state_quaternion.yacc = (int16_t)(*ground_truth[15]);
        hil_state_quaternion.zacc = (int16_t)(*ground_truth[16]);

        mavlink_msg_hil_state_quaternion_encode_chan(1, 200, MAVLINK_COMM_0, &encoded_msg, &hil_state_quaternion);

        bytesToSend += mavlink_msg_to_send_buffer(&buffer[bytesToSend], &encoded_msg);

        auto bytesSent = sock->send(asio::buffer(buffer, bytesToSend));
        
        auto bytesAvailable = sock->available();

        if (bytesAvailable)
        {

            memset(buffer, 0, 1024);

            auto bytesReceived = sock->receive(asio::buffer(buffer, bytesAvailable));

            mavlink_status_t status;

            for (auto i = 0; i < bytesReceived; i++)
            {
                if (mavlink_parse_char(MAVLINK_COMM_0, buffer[i], &encoded_msg, &status))
                {
                    if (encoded_msg.msgid == MAVLINK_MSG_ID_HIL_ACTUATOR_CONTROLS)
                    {
                        mavlink_msg_hil_actuator_controls_decode(&encoded_msg, &hil_actuator_controls_msg);
                    }
                }
            }
        }

        real_T *pwm = ssGetOutputPortRealSignal(S, 0);

        for (auto i = 0; i < 16; i++)
        {
            pwm[i] = (real_T)hil_actuator_controls_msg.controls[i];
        }
    }
    catch (const std::exception &e)
    {
        eStatus = std::string(e.what());
        ssSetErrorStatus(S, eStatus.c_str());
    }
}

/* Function: mdlTerminate =====================================================
 * Abstract:
 *    In this function, you should perform any actions that are necessary
 *    at the termination of a simulation.  For example, if memory was
 *    allocated in mdlStart, this is the place to free it.
 */
static void mdlTerminate(SimStruct *S)
{
    if (ssGetPWork(S) != NULL)
    {
        asio::ip::tcp::socket *sock = (asio::ip::tcp::socket *)(ssGetPWorkValue(S, 0));
        uint8_t *buffer = (uint8_t *)ssGetPWorkValue(S, 1);
        if (sock)
        {
            sock->close();
        }
        if (buffer)
        {
            free(buffer);
        }
    }
}
/*======================================================*
 * See sfuntmpl.doc for the optional S-function methods *
 *======================================================*/

/*=============================*
 * Required S-function trailer *
 *=============================*/

#ifdef MATLAB_MEX_FILE /* Is this file being compiled as a MEX-file? */
#include "simulink.c"  /* MEX-file interface mechanism */
#else
#include "cg_sfun.h" /* Code generation registration function */
#endif