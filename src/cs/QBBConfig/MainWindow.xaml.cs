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

                ulong age = m_task.Status.get_age();
                if (age > 10000)
                    GpsAge.Text = "stale";
                else if ((age & 0x80000000)!=0)
                    GpsAge.Text = "invalid";
                else
                    GpsAge.Text = age.ToString();

                SteeringSP.Text = m_task.Status.get_steering_sp().ToString();
                SteeringPV.Text = m_task.Status.get_steering_pv().ToString();
                SteeringPidErr.Text = m_task.Status.get_steering_pid_err().ToString();

                pb_k1.Value = m_task.Status.get_k1();
                pb_k2.Value = m_task.Status.get_k2();
                pb_k3.Value = m_task.Status.get_k3();
                pb_k4.Value = m_task.Status.get_k4();
                pb_ml.Value = m_task.Status.get_motor_l();
                pb_mr.Value = m_task.Status.get_motor_r();

                TMainState stm = (TMainState)m_task.Status.get_mainseq_step();
                string sStepName = StepToText(stm);
                SeqStep.Text = sStepName;
            }
        }

        private string StepToText(TMainState stm)
        {
            switch (stm)
            {
                case TMainState.msManualMode:
                    return "msManualMode";
                case TMainState.msAutoModeCourse:
                    return "msAutoModeCourse";
                case TMainState.msAutoModeNormal:
                    return "msAutoModeNormal";
                case TMainState.msReverseThrust:
                    return "msReverseThrust";
                case TMainState.msCountJoyGoto:
                    return "msCountJoyGoto";
                case TMainState.msCountJoyGotoRetn:
                    return "msCountJoyGotoRetn";
                case TMainState.msConfirmGotoPosX:
                    return "msConfirmGotoPosX";
                case TMainState.msCountJoyStore:
                    return "msCountJoyStore";
                case TMainState.msCountJoyStoreRetn:
                    return "msCountJoyStoreRetn";
                case TMainState.msConfirmStorePosX:
                    return "msConfirmStorePosX";
                case TMainState.msClear1:
                    return "msClear1";
                case TMainState.msClear2:
                    return "msClear2";
                case TMainState.msConfirmClear:
                    return "msConfirmClear";
                case TMainState.msCmdErrorMan:
                    return "msCmdErrorMan";
                case TMainState.msCmdErrorAuto:
                    return "msCmdErrorAuto";
                case TMainState.msLast:
                    return "msLast";
                default:
                    return "?";
            }
        }
    }
}
