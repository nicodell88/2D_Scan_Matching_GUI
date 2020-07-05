using System;
using Gtk;
using D_Scan_Matching_GUI;

namespace D_Scan_Matching_GUI
{
    class MainClass
    {
        public static void Main(string[] args)
        {
            ScanMatching.Init("../../../TestData.txt");

            //Application.Init();
            //MainWindow win = new MainWindow();
            //win.Show();
            //Application.Run();
        }
    }
}
