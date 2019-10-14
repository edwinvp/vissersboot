using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.IO.Ports;
using System.Linq;
using System.Text;
using System.Threading;
using System.Threading.Tasks;

namespace QBBConfig
{
    class CommTask
    {
        private volatile bool do_stop = false;
        public BbStatus Status = new BbStatus();

        static SerialPort _serialPort;

        public void Stop()
        {
            do_stop = true;
        }

        static float ToFloat(byte[] input)
        {
            byte[] newArray = new[] { input[3], input[2], input[1], input[0] };
            return BitConverter.ToSingle(newArray, 0);
        }

        public float ReadFloat()
        {
            try
            {
                string sLine = _serialPort.ReadLine().Trim();

                byte b3 = Convert.ToByte(sLine.Substring(3,2), 16);
                byte b2 = Convert.ToByte(sLine.Substring(5,2), 16);
                byte b1 = Convert.ToByte(sLine.Substring(7,2), 16);
                byte b0 = Convert.ToByte(sLine.Substring(9,2), 16);

                byte[] byteArray = { b3, b2, b1, b0 };

                return ToFloat(byteArray);

            }
            catch (Exception)
            {
            }
            return 0;
        }

        public long ReadLong()
        {
            try
            {
                string sLine = _serialPort.ReadLine().Trim();
                return Convert.ToInt32(sLine.Substring(3, 8), 16);
            }
            catch (Exception)
            {
            }
            return 0;
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
                    if (!do_stop)
                    {
                        _serialPort.WriteLine("R0002");
                        Status.set_lat(ReadFloat());
                    }

                    if (!do_stop)
                    {
                        _serialPort.WriteLine("R0003");
                        Status.set_lon(ReadFloat());
                    }

                    if (!do_stop)
                    {
                        _serialPort.WriteLine("R0004");
                        Status.set_age(ReadLong());
                    }

                    Thread.Sleep(100);
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
