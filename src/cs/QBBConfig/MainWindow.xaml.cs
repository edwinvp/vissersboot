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
                
        private void set_need_angle(float new_angle)
        {
            RotateTransform rt1 = (RotateTransform)CompassNeedle.RenderTransform;
            rt1.Angle = new_angle;
            RotateTransform rt2 = (RotateTransform)NeedleShadow.RenderTransform;
            rt2.Angle = new_angle;
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

                MagRawX.Text = m_task.Status.get_mag_raw_x().ToString();
                MagRawY.Text = m_task.Status.get_mag_raw_y().ToString();
                MagRawZ.Text = m_task.Status.get_mag_raw_z().ToString();
                MagCourse.Text = m_task.Status.get_mag_course().ToString();
                MagCalState.Text = CalibrationStateToText(m_task.Status.get_mag_cal_state());                
                BtnState.Text = m_task.Status.get_button_state().ToString();

                pb_k1.Value = m_task.Status.get_k1();
                pb_k2.Value = m_task.Status.get_k2();
                pb_k3.Value = m_task.Status.get_k3();
                pb_k4.Value = m_task.Status.get_k4();
                pb_ml.Value = m_task.Status.get_motor_l();
                pb_mr.Value = m_task.Status.get_motor_r();

                set_need_angle(m_task.Status.get_mag_course());

                TMainState stm = (TMainState)m_task.Status.get_mainseq_step();
                string sStepName = StepToText(stm);
                SeqStep.Text = sStepName;
            }
        }

        private string CalibrationStateToText(ECalibrationState cs)
        {
            switch (cs) {
            case ECalibrationState.csNotCalibrated:
                    return "csNotCalibrated";
            case ECalibrationState.csCenterDetect:
                    return "csCenterDetect";
            case ECalibrationState.csTurn1:
                return "csTurn1";
            case ECalibrationState.csTurn2:
                return "csTurn2";
            case ECalibrationState.csFinish:
                return "csFinish";
            case ECalibrationState.csCalibrated:
                return "csCalibrated";
            default:
                return "";
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

        private void MenuItem_Connect(object sender, RoutedEventArgs e)
        {
            try
            {
                if (m_thread != null)
                    throw new Exception("Already connected or in the progress of connecting");

                m_task = new CommTask();
                m_thread = new Thread(m_task.Run);
                m_thread.Start();

                MnuItemConnect.IsEnabled = false;
                MnuItemDisconnect.IsEnabled = true;
            }
            catch (Exception except)
            {
                MessageBox.Show("Can't connect: " + except.Message, "Exception");
            }
        }

        private void MenuItem_Disconnect(object sender, RoutedEventArgs e)
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

                MnuItemConnect.IsEnabled = true;
                MnuItemDisconnect.IsEnabled = false;
            }
            catch (Exception except)
            {
                MessageBox.Show("Can't disconnect: " + except.Message, "Exception");
            }
        }

        private void BtnWriteVar_Click(object sender, RoutedEventArgs e)
        {
            string sAddr = TextAddr.Text.ToString();
            string sNewValue = TextNewValue.Text.ToString();
            int iAddr = int.Parse(sAddr);
            int iNewValue = int.Parse(sNewValue);
            if (m_task != null)
            {
                m_task.WriteLongIndirect((USB_VAR)iAddr, iNewValue);
            }
        }

        private void BtnStartCalibration_Click(object sender, RoutedEventArgs e)
        {
            if (m_task != null)
                m_task.WriteLongIndirect(USB_VAR.urBtnState, 1);
        }

        private void BtnSetTrueNorth_Click(object sender, RoutedEventArgs e)
        {
            if (m_task != null)
                m_task.WriteLongIndirect(USB_VAR.urSetTrueNorth, 1);
        }
    }
}
