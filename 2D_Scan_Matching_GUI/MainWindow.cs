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
        var plotView = new PlotView();
        this.Add(plotView);
        plotView.ShowAll();


        var model = new PlotModel { Title = "ScatterSeries" };
        var scatterSeries = new ScatterSeries { MarkerType = MarkerType.Circle };
        var r = new Random(314);
        for (int i = 0; i < 100; i++)
        {
            var x = r.NextDouble();
            var y = r.NextDouble();
            var size = r.Next(5, 15);
            var colorValue = r.Next(100, 1000);
            scatterSeries.Points.Add(new ScatterPoint(x, y, size, colorValue));
        }

        model.Series.Add(scatterSeries);
        model.Axes.Add(new LinearColorAxis { Position = AxisPosition.Right, Palette = OxyPalettes.Jet(200) });
        plotView.Model = model;
        //Choose file



        // Run scan Matching
        //ScanMatching.RunScanMatch("../../../LaserDataSimulated.txt");


    }

    protected void OnDeleteEvent(object sender, DeleteEventArgs a)
    {
        Application.Quit();
        a.RetVal = true;
    }
}
