﻿using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.IO.Ports;
using System.Linq;
using System.Text;
using System.Threading;
using System.Threading.Tasks;

namespace QBBConfig
{
    public enum TMainState
    {
        msManualMode = 0, // manual control mode
        msAutoModeCourse, // 'course' adjustments
        msAutoModeNormal, // automatic waypoint mode
        msReverseThrust, // reverse thrust after arriving at wp
        msCountJoyGoto, // count joystick 'up' command
        msCountJoyGotoRetn,
        msConfirmGotoPosX,
        msCountJoyStore, // count joystick 'down' command
        msCountJoyStoreRetn,
        msConfirmStorePosX,
        msClear1,
        msClear2,
        msConfirmClear,
        msCmdErrorMan,
        msCmdErrorAuto,
        msLast
    };

    public enum USB_VAR
    {
        urInvalid=0,
        urMagic = 1,
        urGpsLat = 2,
        urGpsLon = 3,
        urGpsAge = 4,
        urGpsValid = 5,
        urGpsCourse = 6,
        urRcK1 = 10,
        urRcK2,
        urRcK3,
        urRcK4,
        urMainSeqStep = 20,
        urMotorL = 30,
        urMotorR = 31,
        urSteeringSP = 40,
        urSteeringPV = 41,
        urSteeringPID_ERR = 42,
        urMagRawX = 50,
        urMagRawY = 51,
        urMagRawZ = 52,
        urMagCourse = 53,
        urMagCalState = 54,
        urMagMinX = 55,
        urMagMaxX = 56,
        urMagMinY = 57,
        urMagMaxY = 58,
        urMagMinZ = 59,
        urMagMaxZ = 60,
        urBtnState = 80,
        urSetTrueNorth,
    };

    class CommTask
    {
        private volatile bool do_stop = false;
        public BbStatus Status = new BbStatus();

        private volatile int wr_addr = 0;
        private volatile int wr_data = 0;
        private volatile bool do_write = false;

        private volatile int m_counter = 0;

        static SerialPort _serialPort;

        public int GetCounter()
        {
            return m_counter;
        }

        public void Stop()
        {
            do_stop = true;
        }

        public void WriteLongIndirect(USB_VAR iAddr, int iValue)
        {
            wr_addr = (int)iAddr;
            wr_data = iValue;
            do_write = true;
        }

        static float ToFloat(byte[] input)
        {
            byte[] newArray = new[] { input[3], input[2], input[1], input[0] };
            return BitConverter.ToSingle(newArray, 0);
        }

        public float UintToFloat(uint uiData)
        {
            byte b3 = (byte)((uiData >> 24) & 255);
            byte b2 = (byte)((uiData >> 16) & 255);
            byte b1 = (byte)((uiData >> 8) & 255);
            byte b0 = (byte)(uiData & 255);

            byte[] byteArray = { b3, b2, b1, b0 };

            return ToFloat(byteArray);
        }

        public void SendWriteCommand(int iAddr, int iData)
        {
            String sHexAddr = iAddr.ToString("X4");
            String sHexData = iData.ToString("X8");
            String sWriteCmd = "W" + sHexAddr + "," + sHexData;
            _serialPort.WriteLine(sWriteCmd);
        }

        public bool CheckWriteResponse()
        {
            try
            {
                string sLine = _serialPort.ReadLine().Trim();
                return true;
            }
            catch (Exception)
            {
            }
            return false;
        }

        public void SetFromRaw(USB_VAR uv, uint uiData)
        {
            switch (uv)
            {
                case USB_VAR.urGpsLat:
                    Status.set_lat(UintToFloat(uiData));
                    break;
                case USB_VAR.urGpsLon:
                    Status.set_lon(UintToFloat(uiData));
                    break;
                case USB_VAR.urGpsAge:
                    Status.set_age(uiData);
                    break;
                case USB_VAR.urRcK1:
                    Status.set_k1((long)uiData);
                    break;
                case USB_VAR.urRcK2:
                    Status.set_k2((long)uiData);
                    break;
                case USB_VAR.urRcK3:
                    Status.set_k3((long)uiData);
                    break;
                case USB_VAR.urRcK4:
                    Status.set_k4((long)uiData);
                    break;
                case USB_VAR.urMainSeqStep:
                    Status.set_mainseq_step((long)uiData);
                    break;
                case USB_VAR.urMotorL:
                    Status.set_motor_l((int)uiData);
                    break;
                case USB_VAR.urMotorR:
                    Status.set_motor_r((int)uiData);
                    break;
                case USB_VAR.urMagRawX:
                    Status.set_mag_raw_x((int)uiData);
                    break;
                case USB_VAR.urMagRawY:
                    Status.set_mag_raw_y((int)uiData);
                    break;
                case USB_VAR.urMagRawZ:
                    Status.set_mag_raw_z((int)uiData);
                    break;
                case USB_VAR.urMagCourse:
                    Status.set_mag_course(UintToFloat(uiData));
                    break;
                case USB_VAR.urMagCalState:
                    Status.set_mag_cal_state((ECalibrationState)uiData);
                    break;
                case USB_VAR.urSteeringSP:
                    Status.set_steering_sp(UintToFloat(uiData));
                    break;
                case USB_VAR.urSteeringPV:
                    Status.set_steering_pv(UintToFloat(uiData));
                    break;
                case USB_VAR.urSteeringPID_ERR:
                    Status.set_steering_pid_err(UintToFloat(uiData));
                    break;
                case USB_VAR.urBtnState:
                    Status.set_button_state((int)uiData);
                    break;

            }

        }

        public void ReadResponse()
        {
            try
            {
                string sLine = _serialPort.ReadLine().Trim();

                int iAddr = Convert.ToInt32(sLine.Substring(4, 4), 16);
                uint uiData = Convert.ToUInt32(sLine.Substring(9, 8), 16);

                SetFromRaw((USB_VAR)iAddr, uiData);

            }
            catch (Exception)
            {
            }
            return;
        }

        public void SendReadCommand(USB_VAR uv)
        {
            int iv = (int)uv;
            String sHex = iv.ToString("X4");
            String sReadCmd = "R" + sHex;
            _serialPort.WriteLine(sReadCmd);
        }

        public void Run(object data)
        {
            try
            {

                Thread.CurrentThread.IsBackground = true;

                // Create a new SerialPort object with default settings.
                _serialPort = new SerialPort();

                // Allow the user to set the appropriate properties.
                _serialPort.PortName = "COM15";
                _serialPort.BaudRate = 9600;
                _serialPort.Parity = Parity.None;
                _serialPort.DataBits = 8;
                _serialPort.StopBits = StopBits.One;
                _serialPort.Handshake = Handshake.None;

                // Set the read/write timeouts
                _serialPort.ReadTimeout = 2000;
                _serialPort.WriteTimeout = 2000;

                Debug.Print("Connect to COM port");

                _serialPort.Open();

                Debug.Print("Comms Opened.");

                while (!do_stop)
                {
                    m_counter++;

                    if (!do_stop)
                    {
                        SendReadCommand(USB_VAR.urGpsLat);
                        SendReadCommand(USB_VAR.urGpsLon);
                        SendReadCommand(USB_VAR.urGpsAge);
                        ReadResponse();
                        ReadResponse();
                        ReadResponse();
                    }

                    if (!do_stop)
                    {
                        SendReadCommand(USB_VAR.urRcK1);
                        SendReadCommand(USB_VAR.urRcK2);
                        SendReadCommand(USB_VAR.urRcK3);
                        SendReadCommand(USB_VAR.urRcK4);
                        ReadResponse();
                        ReadResponse();
                        ReadResponse();
                        ReadResponse();
                    }

                    if (!do_stop)
                    {
                        SendReadCommand(USB_VAR.urMainSeqStep);
                        SendReadCommand(USB_VAR.urMotorL);
                        SendReadCommand(USB_VAR.urMotorR);
                        ReadResponse();
                        ReadResponse();
                        ReadResponse();
                    }
                    if (!do_stop)
                    {
                        SendReadCommand(USB_VAR.urSteeringSP);
                        SendReadCommand(USB_VAR.urSteeringPV);
                        SendReadCommand(USB_VAR.urSteeringPID_ERR);
                        SendReadCommand(USB_VAR.urBtnState);
                        ReadResponse();
                        ReadResponse();
                        ReadResponse();
                        ReadResponse();
                    }

                    if (!do_stop)
                    {
                        SendReadCommand(USB_VAR.urMagRawX);
                        SendReadCommand(USB_VAR.urMagRawY);
                        SendReadCommand(USB_VAR.urMagRawZ);
                        SendReadCommand(USB_VAR.urMagCourse);
                        SendReadCommand(USB_VAR.urMagCalState);
                        ReadResponse();
                        ReadResponse();
                        ReadResponse();
                        ReadResponse();
                        ReadResponse();
                    }

                    if (!do_stop && do_write)
                    {
                        int iAddr = wr_addr;
                        int iData = wr_data;
                        do_write = false;

                        SendWriteCommand(iAddr,iData);
                        if (CheckWriteResponse())
                            Debug.Print("Write succeeded");
                        else
                            Debug.Print("Write failed");
                    }                    

                    Thread.Sleep(50);
                }

                Debug.Print("Stopping. Closing COM-port.");

                if (_serialPort.IsOpen)
                    _serialPort.Close();

                Debug.Print("Comms Closed.");

            }
            catch (Exception e) {
                if (_serialPort.IsOpen)
                    _serialPort.Close();
                _serialPort = null;
                Debug.Print(e.ToString());
            }

        }


    }
}
