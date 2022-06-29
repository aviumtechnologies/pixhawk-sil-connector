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
#include "mavlink/ardupilotmega/mavlink.h"

#define S_FUNCTION_LEVEL 2
// #define S_FUNCTION_DEBUG
#define S_FUNCTION_NAME pixhawk_sil_connector

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
    ssSetInputPortWidth(S, 9, 20); // rc_channels (-)
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
    ssSetSampleTime(S, 0, INHERITED_SAMPLE_TIME);
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
        void **PWork = ssGetPWork(S);

#ifndef S_FUNCTION_DEBUG

        static asio::io_service io_service;
        asio::ip::tcp::endpoint endpoint(asio::ip::address::from_string("0.0.0.0"), 4560);
        asio::ip::tcp::acceptor acceptor(io_service, endpoint);
        static asio::ip::tcp::socket socket(io_service);

        static double pwm[16];
        for (unsigned int i = 0; i < 16; i++)
        {
            pwm[i] = 0.0;
        }
        PWork[0] = &socket;
        PWork[1] = &pwm;

        acceptor.accept(socket);
#endif
    }
    catch (const std::exception &e)
    {
        static char errorStatus[256];
        sprintf(errorStatus, "%s\n", e.what());
        ssSetErrorStatus(S, errorStatus);
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
        void **PWork = ssGetPWork(S);

        asio::ip::tcp::socket *socket = ((asio::ip::tcp::socket *)PWork[0]);
        
        double *pwm = ((double *)PWork[1]);

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

        real_T *pwm_out = ssGetOutputPortRealSignal(S, 0);

        uint8_t buffer[65535];
        mavlink_message_t encoded_msg;
        
        //HIL_SENSOR
        {

#ifdef S_FUNCTION_DEBUG
        mexPrintf("time=%f\n", (*time[0]));
        mexPrintf("A_measured(1)=%f\n", (*A_measured[0]));
        mexPrintf("A_measured(2)=%f\n", (*A_measured[1]));
        mexPrintf("A_measured(3)=%f\n", (*A_measured[2]));
        mexPrintf("omega_b_measured(1)=%f\n", (*omega_b_measured[0]));
        mexPrintf("omega_b_measured(2)=%f\n", (*omega_b_measured[1]));
        mexPrintf("omega_b_measured(3)=%f\n", (*omega_b_measured[2]));
        mexPrintf("B_measured(1)=%f\n", (*B_measured[0]));
        mexPrintf("B_measured(2)=%f\n", (*B_measured[1]));
        mexPrintf("B_measured(3)=%f\n", (*B_measured[2]));
        mexPrintf("P_measured=%f\n", (*P_measured[0]));
        mexPrintf("T_measured=%f\n", (*T_measured[0]));
        mexPrintf("h_measured=%f\n",(*h_measured[0]));
        mexPrintf("q_measured=%f\n",(*q_measured[0]));            
#endif

        mavlink_hil_sensor_t msg;
        msg.time_usec = (uint64_t)((*time[0]) * 1e6);
        msg.xacc = (float)(*A_measured[0]);    
        msg.yacc = (float)(*A_measured[1]);   
        msg.zacc = (float)(*A_measured[2]);    
        msg.xgyro = (float)(*omega_b_measured[0]); 
        msg.ygyro = (float)(*omega_b_measured[1]);
        msg.zgyro = (float)(*omega_b_measured[2]);
        msg.xmag = (float)(*B_measured[0]); 
        msg.ymag = (float)(*B_measured[1]); 
        msg.zmag = (float)(*B_measured[2]);
        msg.abs_pressure = (float)(*P_measured[0]);
        msg.diff_pressure = (float)(*q_measured[0]); 
        msg.pressure_alt = (float)(*h_measured[0]);  
        msg.temperature = (float)(*T_measured[0]); 
        msg.fields_updated = (uint32_t)0x1fff;

        mavlink_msg_hil_sensor_encode_chan(1, 200, MAVLINK_COMM_0, &encoded_msg, &msg);

        auto length = mavlink_msg_to_send_buffer(buffer, &encoded_msg);

#ifndef S_FUNCTION_DEBUG
        if (asio::write(*socket, asio::buffer(buffer, length)) != length)
        {
            std::runtime_error("Error sending HIL_SENSOR message!");
        }
#endif

        }
        
        //HIL_GPS
        {
#ifdef S_FUNCTION_DEBUG
        mexPrintf("time=%f\n", (*time[0]));
        mexPrintf("fix=%f\n", (*xyz_measured[0]));
        mexPrintf("lat=%f\n", (*xyz_measured[1]));
        mexPrintf("lon=%f\n", (*xyz_measured[2]));
        mexPrintf("alt=%f\n", (*xyz_measured[3]));
        mexPrintf("eph=%f\n", (*xyz_measured[4]));
        mexPrintf("epv=%f\n", (*xyz_measured[5]));
        mexPrintf("vel=%f\n", (*xyz_measured[6]));
        mexPrintf("vn=%f\n", (*xyz_measured[7]));
        mexPrintf("ve=%f\n", (*xyz_measured[8]));
        mexPrintf("vd=%f\n", (*xyz_measured[9]));
        mexPrintf("cog=%f\n", (*xyz_measured[10]));        
        mexPrintf("sattelites_visible=%f\n", (*xyz_measured[11]));
#endif
        mavlink_hil_gps_t msg;
        msg.time_usec = (uint64_t)((*time[0]) * 1e6);
        msg.fix_type = (uint8_t)(*xyz_measured[0]);
        msg.lat = (int32_t)(*xyz_measured[1]); 
        msg.lon = (int32_t)(*xyz_measured[2]); 
        msg.alt = (int32_t)(*xyz_measured[3]);  
        msg.eph = (uint16_t)(*xyz_measured[4]); 
        msg.epv = (uint16_t)(*xyz_measured[5]); 
        msg.vel = (uint16_t)(*xyz_measured[6]);
        msg.vn = (int16_t)(*xyz_measured[7]); 
        msg.ve = (int16_t)(*xyz_measured[8]); 
        msg.vd = (int16_t)(*xyz_measured[9]);
        msg.cog = (uint16_t)(*xyz_measured[10]);
        msg.satellites_visible = (uint8_t)(*xyz_measured[11]);
        msg.id=0;

        mavlink_msg_hil_gps_encode_chan(1, 200, MAVLINK_COMM_0, &encoded_msg, &msg);
        auto length = mavlink_msg_to_send_buffer(buffer, &encoded_msg);

#ifndef S_FUNCTION_DEBUG
        if (asio::write(*socket, asio::buffer(buffer, length)) != length)
        {
            std::runtime_error("Error sending HIL_GPS message!");
        }
#endif
        }
        
        //HIL_STATE_QUATERNION
        {   
            
            float phi=(float)(*ground_truth[0]);
            float theta=(float)(*ground_truth[1]);
            float psi=(float)(*ground_truth[2]);
         
            float sphi=std::sin(phi/2.0);
            float stheta=std::sin(theta/2.0);
            float spsi=std::sin(psi/2.0);
            float cphi=std::cos(phi/2.0);
            float ctheta=std::cos(theta/2.0);
            float cpsi=std::cos(psi/2.0);

            float attitude_quaternion[4];
            attitude_quaternion[0]=cpsi*ctheta*cphi + spsi*stheta*sphi;
            attitude_quaternion[1]=cpsi*ctheta*sphi - spsi*stheta*cphi;
            attitude_quaternion[2]=cpsi*stheta*cphi + spsi*ctheta*sphi;
            attitude_quaternion[3]=spsi*ctheta*cphi - cpsi*stheta*sphi;

            mavlink_hil_state_quaternion_t msg;
            msg.attitude_quaternion[0]=attitude_quaternion[0];
            msg.attitude_quaternion[1]=attitude_quaternion[1];
            msg.attitude_quaternion[2]=attitude_quaternion[2];
            msg.attitude_quaternion[3]=attitude_quaternion[3];
            msg.rollspeed=(float)(*ground_truth[3]); 
            msg.pitchspeed=(float)(*ground_truth[4]); 
            msg.yawspeed=(float)(*ground_truth[5]); 
            msg.lat=(int32_t)(*ground_truth[6]);
            msg.lon=(int32_t)(*ground_truth[7]); 
            msg.alt=(int32_t)(*ground_truth[8]);
            msg.vx=(int16_t)(*ground_truth[9]);
            msg.vy=(int16_t)(*ground_truth[10]);
            msg.vz=(int16_t)(*ground_truth[11]);
            msg.ind_airspeed=(uint16_t)(*ground_truth[12]);
            msg.true_airspeed=(uint16_t)(*ground_truth[13]);
            msg.xacc=(int16_t)(*ground_truth[14]);
            msg.yacc=(int16_t)(*ground_truth[15]);
            msg.zacc=(int16_t)(*ground_truth[16]);

            
            mavlink_msg_hil_state_quaternion_encode_chan(1, 200, MAVLINK_COMM_0, &encoded_msg, &msg);
            auto length = mavlink_msg_to_send_buffer(buffer, &encoded_msg);

#ifndef S_FUNCTION_DEBUG
        if (asio::write(*socket, asio::buffer(buffer, length)) != length)
        {
            std::runtime_error("Error sending HIL_STATE_QUATERNION message!");
        }
#endif
        }
        
        //RC_CHANNELS
        {
#ifdef S_FUNCTION_DEBUG
        mexPrintf("time=%f\n", (*time[0]));
        mexPrintf("chancount=%f\n", (*rc_channels[0]));
        mexPrintf("chan1_raw=%f\n", (*rc_channels[1]));
        mexPrintf("chan2_raw=%f\n", (*rc_channels[2]));
        mexPrintf("chan3_raw=%f\n", (*rc_channels[3]));
        mexPrintf("chan4_raw=%f\n", (*rc_channels[4]));
        mexPrintf("chan5_raw=%f\n", (*rc_channels[5]));
        mexPrintf("chan6_raw=%f\n", (*rc_channels[6]));
        mexPrintf("chan7_raw=%f\n", (*rc_channels[7]));
        mexPrintf("chan8_raw=%f\n", (*rc_channels[8]));
        mexPrintf("chan9_raw=%f\n", (*rc_channels[9]));
        mexPrintf("chan10_raw=%f\n", (*rc_channels[10]));
        mexPrintf("chan11_raw=%f\n", (*rc_channels[11]));
        mexPrintf("chan12_raw=%f\n", (*rc_channels[12]));        
        mexPrintf("chan13_raw=%f\n", (*rc_channels[13]));
        mexPrintf("chan14_raw=%f\n", (*rc_channels[14]));
        mexPrintf("chan15_raw=%f\n", (*rc_channels[15]));
        mexPrintf("chan16_raw=%f\n", (*rc_channels[16]));        
        mexPrintf("chan17_raw=%f\n", (*rc_channels[17]));      
        mexPrintf("chan18_raw=%f\n", (*rc_channels[18])); 
        mexPrintf("rssi=%f\n", (*rc_channels[19]));
#endif
        mavlink_rc_channels_t msg;
        msg.time_boot_ms = (uint64_t)((*time[0]) * 1e3);
        msg.chancount = (uint8_t)(*rc_channels[0]);            
        msg.chan1_raw = (uint16_t)(*rc_channels[1]);
        msg.chan2_raw = (uint16_t)(*rc_channels[2]);
        msg.chan3_raw = (uint16_t)(*rc_channels[3]);
        msg.chan4_raw = (uint16_t)(*rc_channels[4]);
        msg.chan5_raw = (uint16_t)(*rc_channels[5]);
        msg.chan6_raw = (uint16_t)(*rc_channels[6]);
        msg.chan7_raw = (uint16_t)(*rc_channels[7]);
        msg.chan8_raw = (uint16_t)(*rc_channels[8]);
        msg.chan9_raw = (uint16_t)(*rc_channels[9]);
        msg.chan10_raw = (uint16_t)(*rc_channels[10]);
        msg.chan11_raw = (uint16_t)(*rc_channels[11]);
        msg.chan12_raw = (uint16_t)(*rc_channels[12]);
        msg.chan13_raw = (uint16_t)(*rc_channels[13]);
        msg.chan14_raw = (uint16_t)(*rc_channels[14]);
        msg.chan15_raw = (uint16_t)(*rc_channels[15]);
        msg.chan16_raw = (uint16_t)(*rc_channels[16]);
        msg.chan17_raw = (uint16_t)(*rc_channels[17]);
        msg.chan18_raw = (uint16_t)(*rc_channels[18]);            
        msg.rssi = (uint8_t)(*rc_channels[19]);
        
        mavlink_msg_rc_channels_encode_chan(1, 200, MAVLINK_COMM_0, &encoded_msg, &msg);
        auto length = mavlink_msg_to_send_buffer(buffer, &encoded_msg);

#ifndef S_FUNCTION_DEBUG
        if (asio::write(*socket, asio::buffer(buffer, length)) != length)
        {
            std::runtime_error("Error sending RC_CHANNELS message!");
        }
#endif

        }

#ifndef S_FUNCTION_DEBUG
        auto availableLength = socket->available();
        if (availableLength)
        {
            asio::read(*socket, asio::buffer(buffer, availableLength));

            mavlink_status_t status;
            for (unsigned int i = 0; i < availableLength; i++)
            {
                if (mavlink_parse_char(MAVLINK_COMM_0, buffer[i], &encoded_msg, &status))
                {
                    if (encoded_msg.msgid == MAVLINK_MSG_ID_HIL_ACTUATOR_CONTROLS)
                    {
                        mavlink_hil_actuator_controls_t msg;
                        mavlink_msg_hil_actuator_controls_decode(&encoded_msg, &msg);
                        for (unsigned int j = 0; j < 16; j++)
                        {
                            pwm[j] = (real_T)msg.controls[j];
                        }
                    }
                }
            }
        }

        for (unsigned int i = 0; i < 16; i++)
        {
            pwm_out[i] = pwm[i];
        }

        
#endif
    }
    catch (const std::exception &e)
    {
        static char errorStatus[256];
        sprintf(errorStatus, "%s\n", e.what());
        ssSetErrorStatus(S, errorStatus);
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
#ifndef S_FUNCTION_DEBUG
    try
    {
        void **PWork = ssGetPWork(S);
        if (PWork[0] != nullptr)
        {
            ((asio::ip::tcp::socket *)PWork[0])->close();
        }
    }
    catch (const std::exception &e)
    {
        static char errorStatus[256];
        sprintf(errorStatus, "%s\n", e.what());
        ssSetErrorStatus(S, errorStatus);
    }
#endif
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
