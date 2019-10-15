using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading;
using System.Threading.Tasks;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Data;
using System.Windows.Documents;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Media.Imaging;
using System.Windows.Navigation;
using System.Windows.Shapes;

namespace QBBConfig
{
    /// <summary>
    /// Interaction logic for MainWindow.xaml
    /// </summary>
    public partial class MainWindow : Window
    {
        private CommTask m_task;
        private Thread m_thread;

        public MainWindow()
        {
            InitializeComponent();

            System.Windows.Threading.DispatcherTimer dispatcherTimer = new System.Windows.Threading.DispatcherTimer();
            dispatcherTimer.Tick += new EventHandler(dispatcherTimer_Tick);
            dispatcherTimer.Interval = new TimeSpan(0, 0, 0,0,250);
            dispatcherTimer.Start();
        }

        private void DisconnectBtn_Click(object sender, RoutedEventArgs e)
        {
            try
            {
                if (m_thread != null)
                {
                    m_task.Stop();
                    m_thread.Join();
                    m_thread = null;
                    m_task = null;
                }

                BtnConnect.IsEnabled = true;
                BtnDisconnect.IsEnabled = false;
            }
            catch (Exception except)
            {
                MessageBox.Show("Can't disconnect: " + except.Message, "Exception");
            }
        }

        private void ConnectBtn_Click(object sender, RoutedEventArgs e)
        {
            try
            {
                if (m_thread != null)
                    throw new Exception("Already connected or in the progress of connecting");

                m_task = new CommTask();
                m_thread = new Thread(m_task.Run);
                m_thread.Start();

                BtnConnect.IsEnabled = false;
                BtnDisconnect.IsEnabled = true;
            }
            catch (Exception except)
            {
                MessageBox.Show("Can't connect: " + except.Message, "Exception");
            }
        }

        private void dispatcherTimer_Tick(object sender, EventArgs e)
        {
            // code goes here
            if (m_task != null) {
                GpsLat.Text = m_task.Status.get_lat().ToString();
                GpsLon.Text = m_task.Status.get_lon().ToString();
                GpsAge.Text = m_task.Status.get_age().ToString();
                pb_k1.Value = m_task.Status.get_k1();
                pb_k2.Value = m_task.Status.get_k2();
                pb_k3.Value = m_task.Status.get_k3();
                pb_k4.Value = m_task.Status.get_k4();
            }
        }

    }
}
