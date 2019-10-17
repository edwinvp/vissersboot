using System;
using System.Collections.Generic;
using System.IO.Ports;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Data;
using System.Windows.Documents;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Media.Imaging;
using System.Windows.Shapes;

namespace QBBConfig
{
    /// <summary>
    /// Interaction logic for PortSelectionWnd.xaml
    /// </summary>
    public partial class PortSelectionWnd : Window
    {
        private string SelectedComPort;

        public string GetSelectedComPort()
        {
            return SelectedComPort;
        }

        public PortSelectionWnd()
        {
            SelectedComPort = "";

            InitializeComponent();

            string[] ports = SerialPort.GetPortNames();

            // Display each port name to the console.
            foreach (string port in ports)
            {
                ComboPortNames.Items.Add(port);
            }

            if (ComboPortNames.Items.Count > 0)
                ComboPortNames.SelectedIndex = ComboPortNames.Items.Count-1;
            else
                MessageBox.Show("No serial ports found on your system!");

        }

        private void OkBtn_Click(object sender, RoutedEventArgs e)
        {
            SelectedComPort = ComboPortNames.SelectedValue.ToString();
            DialogResult = true;
            Close();
        }

        private void CancelBtn_Click(object sender, RoutedEventArgs e)
        {
            DialogResult = false;
            Close();
        }
    }
}
