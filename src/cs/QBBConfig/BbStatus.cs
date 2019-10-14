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

    }
}
