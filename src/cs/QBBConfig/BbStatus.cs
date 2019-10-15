using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace QBBConfig
{
    enum ECalibrationState
    {
        csNotCalibrated=0, csCenterDetect, csTurn1, csTurn2, csFinish, csCalibrated
    };

    class BbStatus
    {
        private float m_lat = 0;
        private float m_lon = 0;
        private ulong m_age = 0;
        private long m_k1 = 0;
        private long m_k2 = 0;
        private long m_k3 = 0;
        private long m_k4 = 0;
        private long m_mainseq_step = -1;
        private long m_motor_l = 0;
        private long m_motor_r = 0;
        private float m_steering_sp = 0;
        private float m_steering_pv = 0;
        private float m_steering_pid_err = 0;
        private long m_mag_raw_x = 0;
        private long m_mag_raw_y = 0;
        private long m_mag_raw_z = 0;
        private float m_mag_course = 0.0f;
        private ECalibrationState m_mag_cal_state = 0;

        private Object Lck = new Object();

        public void set_lat(float lat)
        {
            lock (Lck)
            {
                m_lat = lat;
            }
        }

        public void set_lon(float lon)
        {
            lock (Lck)
            {
                m_lon = lon;
            }
        }

        public void set_age(ulong age)
        {
            lock (Lck)
            {
                m_age = age;
            }
        }

        public void set_k1(long k1)
        {
            lock (Lck)
            {
                m_k1 = k1;
            }
        }

        public void set_k2(long k2)
        {
            lock (Lck)
            {
                m_k2 = k2;
            }
        }

        public void set_k3(long k3)
        {
            lock (Lck)
            {
                m_k3 = k3;
            }
        }

        public void set_k4(long k4)
        {
            lock (Lck)
            {
                m_k4 = k4;
            }
        }

        public void set_mainseq_step(long s)
        {
            lock (Lck)
            {
                m_mainseq_step = s;
            }
        }

        public void set_motor_l(long s)
        {
            lock (Lck)
            {
                m_motor_l = s;
            }
        }

        public void set_motor_r(long s)
        {
            lock (Lck)
            {
                m_motor_r = s;
            }
        }

        public void set_steering_sp(float f)
        {
            lock (Lck)
            {
                m_steering_sp = f;
            }
        }

        public void set_steering_pv(float f)
        {
            lock (Lck)
            {
                m_steering_pv = f;
            }
        }

        public void set_steering_pid_err(float f)
        {
            lock (Lck)
            {
                m_steering_pid_err = f;
            }
        }

        public void set_mag_raw_x(long d)
        {
            lock (Lck)
            {
                m_mag_raw_x = d;
            }
        }

        public void set_mag_raw_y(long d)
        {
            lock (Lck)
            {
                m_mag_raw_y = d;
            }
        }

        public void set_mag_raw_z(long d)
        {
            lock (Lck)
            {
                m_mag_raw_z = d;
            }
        }

        public void set_mag_course(float f)
        {
            lock (Lck)
            {
                m_mag_course = f;
            }
        }

        public void set_mag_cal_state(ECalibrationState d)
        {
            lock (Lck)
            {
                m_mag_cal_state = d;
            }
        }

        public float get_lat()
        {
            lock (Lck)
            {
                return m_lat;
            }
        }

        public float get_lon()
        {
            lock (Lck)
            {
                return m_lon;
            }
        }

        public ulong get_age()
        {
            lock (Lck)
            {
                return m_age;
            }
        }

        public long get_k1()
        {
            lock (Lck)
            {
                return m_k1;
            }
        }

        public long get_k2()
        {
            lock (Lck)
            {
                return m_k2;
            }
        }

        public long get_k3()
        {
            lock (Lck)
            {
                return m_k3;
            }
        }

        public long get_k4()
        {
            lock (Lck)
            {
                return m_k4;
            }
        }

        public long get_mainseq_step()
        {
            lock (Lck)
            {
                return m_mainseq_step;
            }
        }

        public long get_motor_l()
        {
            lock (Lck)
            {
                return m_motor_l;
            }
        }

        public long get_motor_r()
        {
            lock (Lck)
            {
                return m_motor_r;
            }
        }

        public float get_steering_sp()
        {
            lock (Lck)
            {
                return m_steering_sp;
            }
        }

        public float get_steering_pv()
        {
            lock (Lck)
            {
                return m_steering_pv;
            }
        }

        public float get_steering_pid_err()
        {
            lock (Lck)
            {
                return m_steering_pid_err;
            }
        }

        public long get_mag_raw_x()
        {
            lock (Lck)
            {
                return m_mag_raw_x;
            }
        }

        public long get_mag_raw_y()
        {
            lock (Lck)
            {
                return m_mag_raw_y;
            }
        }

        public long get_mag_raw_z()
        {
            lock (Lck)
            {
                return m_mag_raw_z;
            }
        }

        public float get_mag_course()
        {
            lock (Lck)
            {
                return m_mag_course;
            }
        }

        public ECalibrationState get_mag_cal_state()
        {
            lock (Lck)
            {
                return m_mag_cal_state;
            }
        }

    }
}
