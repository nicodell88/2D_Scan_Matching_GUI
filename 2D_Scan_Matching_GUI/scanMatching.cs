using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using MathNet.Numerics.LinearAlgebra;
using MathNet.Numerics.LinearAlgebra.Double;
using MathNet.Numerics.Optimization;
using MathNet.Numerics.Optimization.TrustRegion;


namespace D_Scan_Matching_GUI
{
    /*
     * This class is used for evaluating a 2x2 rotation matrix and its derivative
     */
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

    /*
     * This class contains the cost function, optimisation calls to the optimisation routine, and other helper functions.
     */
    public static class ScanMatching
    {

        public static Matrix<double> rPNn { get; set; } = Matrix<double>.Build.Dense(2, 1); //point cloud

        public static Matrix<double> rEBb { get; set; } = Matrix<double>.Build.Dense(2, 1); //measurements

        public static List<List<double>> Range { get; set; } = new List<List<double>>(); //stores all measurements

        public static Matrix<double> rQBb { get; set; } //a matrix, where each column holds the unit vector directions of each lidar beam in body coordinates

        public static double MaxRange { get; set; } = 60;//metres

        public static List<double[]> Pose { get; set; } = new List<double[]>();//holds pose initial conditions from data file.


        private static readonly double Sigma = 0.1;

        /*
         * Cost Function:
         * Input:
         *  v = (x,y,psi)
         * Output:
         *  cost     f (double)
         *  gradient g (vector)
         *  hessian  h (matrix)
         */
        public static Tuple<double, Vector<double>, Matrix<double>> Cost(Vector<double> v)
        {
            double f = 0;
            var NewVec = Vector<double>.Build;
            var NewMat = Matrix<double>.Build;

            var g = NewVec.Dense(3);
            var h = NewMat.Dense(3, 3);



            var rBNn = v.SubVector(0, 2);
            var rEBn = SO2.EulerRotation(v.At(2)).Multiply(rEBb);
            var dRnb = SO2.Derivative(v.At(2));
            var rENn = NewMat.Dense(2, rEBn.ColumnCount);
            for (int i = 0; i < rENn.ColumnCount; i++)
            {
                rENn.SetColumn(i, rBNn.Add(rEBn.Column(i)));
            }

            double[] Je_tmp = { 1, 0, 0, 1, 0, 0 };
            var Je = NewMat.DenseOfColumnMajor(2, 3, Je_tmp);

            //Get point cloud data.
            for (int i = 0; i < rEBb.ColumnCount; i++)
            {
                //TODO: inner forloop can be vectorised.
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
                    g.Subtract(J.Multiply(r), g); // J'*r
                    h.Subtract(J.OuterProduct(J), h); //J'*J (Gauss-Newton approximation)
                }
            }
            //h = Matrix<double>.Build.DenseIdentity(3);
            return Tuple.Create(0.5 * f / rPNn.ColumnCount, g.Divide((double)rPNn.ColumnCount), h.Divide((double)rPNn.ColumnCount));
        }

        /*
         * Input:
         *  Data file name
         * Output: 
         *  none
         *  
         *  Parse input data and loops over scans, performing scan matching
         */
        public static void RunScanMatch(String inpF)
        {
            //Call Init
            Init(inpF);
            //Initialise Optimisation routine
            var obj = ObjectiveFunction.GradientHessian(Cost);
            var solver = new ConjugateGradientMinimizer(1e0, 30); //(1e-5, 100, false);

            Vector<double> x_init = Vector<double>.Build.DenseOfArray(new double[] { 0.0, 0.0, 0.0 });
            Vector<double> Xopt = Vector<double>.Build.DenseOfArray(new double[] { 0.0, 0.0, 0.0 });

            //Initialise reference point cloud, expressed in map coordinates. rPNn
            var rBNn = Vector<double>.Build.Dense(2);
            rBNn[0] = Pose[0][0];
            rBNn[1] = Pose[0][1];
            var Rnb = SO2.EulerRotation(Pose[0][2]);
            rEBb = GetPointCloudFromRange(Range[0]);
            var rPNn_new = Matrix<double>.Build.DenseOfMatrix(rEBb);

            for (int j = 0; j < rPNn_new.ColumnCount; j++)
            {
                rPNn_new.SetColumn(j, rBNn.Add(Rnb.Multiply(rEBb.Column(j))));
            }

            rPNn = rPNn_new;

            //Loop through data, setting up and running optimisation routine each time.
            for (int i = 1; i < Range.Count(); i++)
            {
                //Initialise independent point cloud, expressed in body coordinates.rPBb
                rEBb = GetPointCloudFromRange(Range[i]);
                //Set up initial conditions
                x_init.SetValues(Pose[i]);

                //Solve
                var result = solver.FindMinimum(obj, x_init);
                Xopt = result.MinimizingPoint;
                rBNn = Vector<double>.Build.Dense(2);
                rBNn[0] = Xopt[0];
                rBNn[1] = Xopt[1];
                Rnb = SO2.EulerRotation(Xopt[2]);

                //Append to PointCloud
                rPNn_new = Matrix<double>.Build.DenseOfMatrix(rEBb);
                for (int j = 0; j < rPNn_new.ColumnCount; j++)
                {
                    rPNn_new.SetColumn(j, rBNn.Add(Rnb.Multiply(rEBb.Column(j))));
                }
                rPNn = rPNn.Append(rPNn_new);
            }

        }

        /*
         * Input: 
         *  List of doubles (ranges)
         * Output:
         *  rEBb (marix): the end point of each beam expressed in body coordinates.
         */
        public static Matrix<double> GetPointCloudFromRange(List<double> inp)
        {
            var idx_ = from item in inp
                       select item < MaxRange;

            // Pick the indicies that are not max Range hits
            List<int> idx = new List<int>();
            for (int i = 0; i < idx_.ToList().Count(); i++)
            {
                if (idx_.ToList()[i])
                {
                    idx.Add(i);
                }
            }


            double[] tmp = new double[idx.Count() * 2];
            for (int i = 0; i < idx.Count(); i++)
            {
                tmp[0 + 2 * i] = rQBb[0, idx[i]] * inp[idx[i]];
                tmp[1 + 2 * i] = rQBb[1, idx[i]] * inp[idx[i]];
            }

            return Matrix<double>.Build.DenseOfColumnMajor(2, idx.Count(), tmp);
        }

        /*
         * Parses data
         */
        public static void Init(String inpF)
        {
            //Read data from file
            List<string> data = new List<string>();

            using (var reader = new StreamReader(inpF))
            {
                while (!reader.EndOfStream)
                {
                    var line = reader.ReadLine();
                    data.Add(line);
                }
            }

            //Parse each line
            foreach (var item in data)
            {
                Console.WriteLine(item);
                var line = item.Split(',');

                switch (line[0])
                {
                    case "LidarAngles":
                        // Read in Lidar Properties
                        List<double> LidarAngles = new List<double>();
                        for (int i = 1; i < line.Length; i++)
                        {
                            LidarAngles.Add(double.Parse(line[i]));
                        }

                        // Convert to unit vectors rQBb
                        rQBb = Matrix<double>.Build.Dense(2, LidarAngles.Count);

                        for (int i = 0; i < rQBb.ColumnCount; i++)
                        {
                            rQBb[0, i] = Math.Cos(LidarAngles[i]);
                            rQBb[1, i] = Math.Sin(LidarAngles[i]);
                        }
                        break;
                    case "LidarMaxRange":
                        MaxRange = double.Parse(line[1]);
                        break;
                    case "ODOM":

                        double[] tmp =  {double.Parse(line[1]),
                        double.Parse(line[2]),
                        double.Parse(line[3])};

                        Pose.Add(tmp);
                        break;
                    case "Lidar":
                        List<double> _ranges = new List<double>();
                        for (int i = 1; i < line.Length; i++)
                        {
                            _ranges.Add(double.Parse(line[i]));
                        }
                        Range.Add(_ranges);
                        break;
                    default:
                        break;
                }

            }//endfor

        }





    }





}


