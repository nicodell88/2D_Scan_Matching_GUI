using System;
using System.Collections.Generic;

using MathNet.Numerics.LinearAlgebra;
using MathNet.Numerics.LinearAlgebra.Double;
using MathNet.Numerics.Optimization;
using MathNet.Numerics.Optimization.TrustRegion;


namespace D_Scan_Matching_GUI
{
    class SO2
    {
        public static Matrix<double> EulerRotation(double Phi)
        {
            double c = Math.Cos(Phi);
            double s = Math.Sin(Phi);
            double[] R = { c, s, -s, c };
            return Matrix<double>.Build.DenseOfColumnMajor(2, 2, R);
        }

        public static Matrix<double> Derivative(double Phi)
        {
            double[] s = { 0, 1, -1, 0 };
            var S = Matrix<double>.Build.DenseOfColumnMajor(2, 2, s);
            return S.Multiply(EulerRotation(Phi));
        }
    }

    public static class ScanMatching
    {

        private static Matrix<double> _rPNn = Matrix<double>.Build.Dense(2, 1);
        private static Matrix<double> _rEBb = Matrix<double>.Build.Dense(2, 1);
        private static readonly double Sigma = 1;

        public static (double, Vector<double>, Matrix<double>) Cost(Vector<double> v)
        //public static Vector<double> cost()
        {
            //var f = new Func<Vector<double>, double>(v => Math.Pow(v[0], 2) + Math.Pow(v[1], 4) + Math.Pow(v[2], 6));              // define function
            //var g = new Func<Vector<double>, Vector<double>>(v => new DenseVector(new[] { 2.0 * v[0], 4.0 * v[1], 6.0 * v[2] }));  // define grandient
            //var obj = ObjectiveFunction.Gradient(f, g);
            //var solver = new TrustRegionNewtonCGMinimizer(1e-5, 1e-5, 1e-5, 1000);
            //var result = solver.FindMinimum((IObjectiveModel)obj, new DenseVector(new[] { 15.0, 15.0, 15.0 })); // initial estimate = (15,15,15)

            //IObjectiveFunction GradientHessian(Func<Vector<double>, double> function, Func<Vector<double>, Vector<double>> gradient, Func<Vector<double>, Matrix<double>> hessian)

            double f = 0;
            var G = Vector<double>.Build;
            var H = Matrix<double>.Build;

            var g = G.Dense(3);
            var h = H.Dense(3, 3);


            //var rENn = Matrix<double>.Build.Dense(2,1);

            var rBNn = v.SubVector(0, 2);
            var rEBn = SO2.EulerRotation(v.At(2)).Multiply(_rEBb);
            var rENn = Matrix<double>.Build.Dense(2, rEBn.ColumnCount);
            for (int i = 0; i < rENn.ColumnCount; i++)
            {
                rENn.SetColumn(i, rBNn.Add(rEBn.Column(i)));
            }

            //Get point cloud data.
            for (int i = 0; i < _rEBb.ColumnCount; i++)
            {
                for (int j = 0; j < _rPNn.ColumnCount; j++)
                {
                    var e = rENn.Column(i).Subtract(_rPNn.Column(j));

                    f += 1.0 / Math.Sqrt(2 * Math.PI * Sigma * Sigma) * Math.Exp(-1.0 / (2 * Sigma * Sigma) * e.DotProduct(e));
                }
            }





            return (-f, g, h);
        }




        public static Matrix<double> rPNn
        {
            get { return _rPNn; }
            set { _rPNn = value; }
        }

        public static Matrix<double> rEBb
        {
            get { return _rEBb; }
            set { _rEBb = value; }
        }



        public static void Init(String inpF)
        {
            //Read data from file

            // Is this a good way to store the point cloud data?
            //List<Matrix<double>> Foobar= new List<Matrix<double>>();

            //Parse data

            //Initialise reference point cloud, expressed in map coordinates. rPNn

            //Initialise independent point cloud, expressed in body coordinates.rPBb

            //

        }



    }



}


