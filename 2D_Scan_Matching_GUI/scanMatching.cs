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
        private static readonly double Sigma = 1;

        public static Tuple< double, Vector<double>, Matrix<double> > Cost(Vector<double> v)
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
            var rEBn = SO2.EulerRotation(v.At(2)).Multiply(rEBb);
            var dRnb = SO2.Derivative(v.At(2));
            var rENn = Matrix<double>.Build.Dense(2, rEBn.ColumnCount);
            for (int i = 0; i < rENn.ColumnCount; i++)
            {
                rENn.SetColumn(i, rBNn.Add(rEBn.Column(i)));
            }

            double[] Je_tmp = { 1, 0, 0, 1, 0, 0 };
            var Je = Matrix<double>.Build.DenseOfColumnMajor(2, 3, Je_tmp);

            //Get point cloud data.
            for (int i = 0; i < rEBb.ColumnCount; i++)
            {
                for (int j = 0; j < rPNn.ColumnCount; j++)
                {
                    //Calculate error
                    var e = rENn.Column(i).Subtract(rPNn.Column(j));
                    //Calculate residual
                    double r = Math.Exp(-0.25 / (Sigma * Sigma) * e.DotProduct(e));

                    //Calculate Jacobian of the exponent
                    Je.SetColumn(2, dRnb.Multiply(rEBb.Column(i)));
                    //Calculate Jacobian of the residual
                    var J = Je.TransposeThisAndMultiply(e).Multiply(-0.5 / (Sigma * Sigma) * r);

                    //Calculate cost
                    f -= r * r;
                    //Calculate the exact gradient and appproximate hessian
                    g = g.Subtract(J.Multiply(r)); // J'*r
                    h = h.Subtract(J.OuterProduct(J)); //J'*J (Gauss-Newton approximation)
                }
            }

            return Tuple.Create(0.5 * f, g, h);
        }




        public static Matrix<double> rPNn { get; set; } = Matrix<double>.Build.Dense(2, 1);

        public static Matrix<double> rEBb { get; set; } = Matrix<double>.Build.Dense(2, 1);



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


