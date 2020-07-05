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
        private static readonly double Sigma = 0.15;

        public static Tuple<double, Vector<double>, Matrix<double>> Cost(Vector<double> v)
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
                    g.Subtract(J.Multiply(r),g); // J'*r
                    h.Subtract(J.OuterProduct(J),h); //J'*J (Gauss-Newton approximation)
                }
            }
            //h = Matrix<double>.Build.DenseIdentity(3);
            return Tuple.Create(0.5 * f, g, h);
        }




        public static Matrix<double> rPNn { get; set; } = Matrix<double>.Build.Dense(2, 1);

        public static Matrix<double> rEBb { get; set; } = Matrix<double>.Build.Dense(2, 1);

        public static List<List<double>> Range { get; set; } = new List<List<double>>();

        public static Matrix<double> rQBb { get; set; } //a matrix, where each column holds the unit vector directions of each lidar beam in body coordinates

        public static double MaxRange { get; set; } = 60;//metres

        public static List<double[]> Pose { get; set; } = new List<double[]>();

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
                        //

                        //for (int i = 0; i < rQBb.ColumnCount; i++)
                        //{
                        //    Console.WriteLine($"{LidarAngles[i]}: {rQBb[0, i]},{rQBb[1, i]}");
                        //}

                        break;
                    case "LidarMaxRange":
                        //Console.WriteLine(MaxRange);
                        MaxRange = double.Parse(line[1]);
                        //Console.WriteLine(MaxRange);
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
             //Data has been parsed

            //Initialise Scan matching






            //Console.WriteLine(GetPointCloudFromRange(Range[0]).ToString());


            //foreach (var ranges_ in Range)
            //{
            //    foreach (var range in ranges_)
            //    {
            //        Console.WriteLine(range);
            //    }
            //}

            // Is this a good way to store the point cloud data?
            //List<Matrix<double>> Foobar= new List<Matrix<double>>();

            //Parse data
            //

        }

        public static void RunScanMatch(String inpF)
        {
            //Call Init
            Init(inpF);
            //Initialise Optimisation routine
            var obj = ObjectiveFunction.GradientHessian(Cost);
            var solver = new ConjugateGradientMinimizer(2e1, 30); //(1e-5, 100, false);

            Vector<double> x_init = Vector<double>.Build.DenseOfArray(new double[] { 0.0,0.0,0.0 });
            Vector<double> Xopt = Vector<double>.Build.DenseOfArray(new double[] { 0.0, 0.0, 0.0 });
            //Initialise reference point cloud, expressed in map coordinates. rPNn
            //var tmp = GetPointCloudFromRange(Range[0]);

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
                //x_init = ; //reuse last optimum as inital conditions
                x_init.SetValues(Pose[i]);
                 var result = solver.FindMinimum(obj, x_init);
                Xopt = result.MinimizingPoint;
                rBNn = Vector<double>.Build.Dense(2);
                rBNn[0] = Xopt[0];
                rBNn[1] = Xopt[1];
                Rnb = SO2.EulerRotation(Xopt[2]);
                //Append to PointCloud
                

                for (int j = 0; j < rPNn_new.ColumnCount; j++)
                {
                    rPNn_new.SetColumn(j, rBNn.Add(Rnb.Multiply(rEBb.Column(j))));
                }
                rPNn = rPNn.Append(rPNn_new);
            }

        }

        public static Matrix<double> GetPointCloudFromRange(List<double> inp)
        {
            //var tmp = Matrix<double>.Build.Dense(2, 1)
            var idx_ = from item in inp
                       select item < MaxRange;

            // Pick the indicies that are not max Range hits
            List<int> idx = new List<int>();
            for (int i = 0; i < idx_.ToList().Count(); i++)
            {
                if(idx_.ToList()[i])
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





    }





}


