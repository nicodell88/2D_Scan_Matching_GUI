using System;
using D_Scan_Matching_GUI;
using Xunit;

using MathNet.Numerics.LinearAlgebra;
using MathNet.Numerics.Differentiation;
//using MathNet.Numerics.Distributions;
using MathNet.Numerics.Optimization;
namespace Scan_Matching_GUI.test
{
    public class TestCostFunction
    {
        [Fact]
        public void CloserPointsImproveCost()
        {
            // Initialise point clouds
            double[] x1 = { 0, 1 };
            double[] x2 = { 0, 2 };

            Matrix<double> PointCloud1 = Matrix<double>.Build.DenseOfColumnMajor(2, 1, x1);
            Matrix<double> PointCloud2 = Matrix<double>.Build.DenseOfColumnMajor(2, 1, x2);

            ScanMatching.rEBb = PointCloud1;
            ScanMatching.rPNn = PointCloud2;

            // Generate two pose vectors (x,y,psi)
            Vector<double> v1 = Vector<double>.Build.Dense(3);//This should yield a higher cost than    
            double[] vv = { 0, 1, 0 };
            Vector<double> v2 = Vector<double>.Build.DenseOfArray(vv);//this one

            (double CostExpectLow, var G1, var H1) = ScanMatching.Cost(v2);
            (double CostExpectHigh, var G2, var H2) = ScanMatching.Cost(v1);

            // Since the points lie on top of one another when pose = {0 1 0} that should yield a lower cost


            Assert.True(CostExpectLow < CostExpectHigh, "Expected closer points to yield a lower cost, something is fundamentally wrong with the cost function");

        }
        [Fact]
        /*The following scenarios are contrived such that the point cloud locations are mirrored and should produce the same cost*/
        public void EqualCostScenarios()
        {
            // Initialise point clouds
            double[] x1 = { 0, 0, 0, 1 };
            double[] x2 = { 0, -0.5, 0, 0.5 };

            Matrix<double> PointCloud1 = Matrix<double>.Build.DenseOfColumnMajor(2, 2, x1);
            Matrix<double> PointCloud2 = Matrix<double>.Build.DenseOfColumnMajor(2, 2, x2);

            ScanMatching.rEBb = PointCloud2;
            ScanMatching.rPNn = PointCloud1;

            // Generate two pose vectors (x,y,psi)
            double[] vv1 = { -0.25, 0.5, -0.1 };
            Vector<double> v1 = Vector<double>.Build.Dense(vv1);

            double[] vv2 = { 0.25, 0.5, 0.1 };
            Vector<double> v2 = Vector<double>.Build.DenseOfArray(vv2);

            (double expected, var G1, var H1) = ScanMatching.Cost(v2);
            (double actual, var G2, var H2) = ScanMatching.Cost(v1);

            /*Precision is given as an integer defining the number of decimal places, 
             * Normal practice would be to select precision based on machine precision
             * E.g., sqrt(eps) where eps is machine precision, sqrt(eps) = 8.5e-8 on my machine.
             * Therefore, 8 decimal places will suffice
             */
            Assert.Equal(expected, actual, 8);
        }

        [Fact]
        /*Check that the returned gradient is equal to the numerical gradient*/
        public void GradientMatchNumerical()
        {
            //
            var v = Vector<double>.Build.Dense(3);
            //
            ScanMatching.rEBb = Matrix<double>.Build.Random(2, 50);
            ScanMatching.rPNn = Matrix<double>.Build.Random(2, 15);

            var hp = new HelperFunctions();
            Func<double[], double> f = x => hp.ExtractCost(x);

            var derivEst = new NumericalJacobian(5,2);
            var gradExpect = derivEst.Evaluate(f,v.ToArray());
            (var ftmp, var gradActual, var Htmp) = ScanMatching.Cost(v);



            // Not sure how to do assert array equal with precision.
            for (int i = 0; i < gradExpect.Length; i++)
            {
                Assert.Equal(gradExpect[i], gradActual.ToArray()[i], 8);
            }

        }

        [Fact]
        public void OptimiserFindMinimum()
        {
            //ScanMatching.rEBb = Matrix<double>.Build.Random(2, 50);
            //ScanMatching.rPNn = Matrix<double>.Build.Random(2, 15);

            //var obj = ObjectiveFunction.GradientHessian(ScanMatching.Cost);
            //var solver = new NewtonMinimizer(1,1,false);
            //var result = solver.FindMinimum((IObjectiveModel)obj, new DenseVector(new[] { 15.0, 15.0, 15.0 })); // initial estimate = (15,15,15)
        }
    }

    public class HelperFunctions
    {
        public double ExtractCost(double[] v)
        {

            var vv = Vector<double>.Build.DenseOfArray(v);

            (var f, var g, var h) = ScanMatching.Cost(vv);

            return f;
        }
    }
}
