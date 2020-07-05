using System;
using D_Scan_Matching_GUI;

using Gtk;
using OxyPlot;
using OxyPlot.GtkSharp;
using OxyPlot.Series;
using OxyPlot.Axes;


public partial class MainWindow : Gtk.Window
{
    public MainWindow() : base(Gtk.WindowType.Toplevel)
    {
        Build();

        //InitialisePlot
        //var plotView = new PlotView();
        //this.Add(plotView);
        //plotView.ShowAll();

        //var myModel = new PlotModel { Title = "Scan Matching" };


        //var scat = new ScatterSeries { MarkerType = MarkerType.Circle };

        //scat.Points.Add()


        //myModel.Series.Add(scat);
        //plotView.Model = myModel;



        //Choose file



        // Run scan Matching
        ScanMatching.RunScanMatch("../../../LaserDataSimulated.txt");

        // Run Plot
        var plotView = new PlotView();
        this.Add(plotView);
        plotView.ShowAll();


        var model = new PlotModel { Title = "ScatterSeries" };
        var scatterSeries = new ScatterSeries { MarkerType = MarkerType.Circle };
        var r = new Random(314);
        for (int i = 0; i < D_Scan_Matching_GUI.ScanMatching.rPNn.ColumnCount; i++)
        {
            var x = D_Scan_Matching_GUI.ScanMatching.rPNn[0,i];
            var y = D_Scan_Matching_GUI.ScanMatching.rPNn[1, i];
            var size = 5;
            var colorValue = 1500;
            scatterSeries.Points.Add(new ScatterPoint(x, y, size, colorValue));
        }

        model.Series.Add(scatterSeries);
        model.Axes.Add(new LinearColorAxis { Position = AxisPosition.Right, Palette = OxyPalettes.Jet(200) });
        plotView.Model = model;


    }

    protected void OnDeleteEvent(object sender, DeleteEventArgs a)
    {
        Application.Quit();
        a.RetVal = true;
    }
}
