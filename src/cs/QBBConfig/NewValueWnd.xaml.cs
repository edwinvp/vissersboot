using System;
using System.Collections.Generic;
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
    /// Interaction logic for NewValueWnd.xaml
    /// </summary>
    public partial class NewValueWnd : Window
    {
        private float NewValue = 0;

        public NewValueWnd()
        {
            InitializeComponent();
        }

        private void BtnOk_Click(object sender, RoutedEventArgs e)
        {
            string sNewString = editValue.Text;
            if (!float.TryParse(sNewString,out NewValue))
            {
                MessageBox.Show("Please enter a valid number", "Parse error");
                return;
            }

            DialogResult = true;
            Close();
        }

        private void BtnCancel_Click(object sender, RoutedEventArgs e)
        {
            DialogResult = false;
        }

        public float GetValue()
        {
            return NewValue;
        }
    }
}
