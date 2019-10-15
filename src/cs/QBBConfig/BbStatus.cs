using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace QBBConfig
{
    class BbStatus
    {
        private float m_lat;
        private float m_lon;
        private long m_age;
        private long m_k1;
        private long m_k2;
        private long m_k3;
        private long m_k4;

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

        public void set_age(long age)
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

        public long get_age()
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

    }
}
