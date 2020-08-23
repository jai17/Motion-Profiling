using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

using GeometryLibraries;
using ProfilingLibraries;

namespace ProfilingLibraries
{
    public class SegmentedSplineGenerator
    {

        private const double kMaxDX = 2.0; // inches
        private const double kMaxDY = 0.25; // inches,  0.05
        private const double kMaxDTheta = 5 * (Math.PI / 180.0); // radians, 0.1
        private const int kMinSampleSize = 1;

        /**
        * Converts a spline into a list of segmented poses
        *
        * @param s  the spline to parametrize
        * @param t0 starting percentage of spline to parametrize
        * @param t1 ending percentage of spline to parametrize
        * @return list of Pose2dWithCurvature that approximates the original spline
        */
        public static List<Pose2dWithCurvature> parameterizeSpline(Spline s, double maxDx, double maxDy, double maxDTheta,
                double t0, double t1)
        {
            List<Pose2dWithCurvature> rv = new List<Pose2dWithCurvature>();
            rv.Add(s.getPose2dWithCurvature(0.0));
            double dt = (t1 - t0);
            for (double t = 0; t < t1; t += dt / kMinSampleSize)
            {
                getSegmentArc(s, ref rv, t, t + dt / kMinSampleSize, maxDx, maxDy, maxDTheta);
            }
            return rv;
        }

        /**
         * Convenience function to parametrize a spline from t 0 to 1
         */
        public static List<Pose2dWithCurvature> parameterizeSpline(Spline s, double maxDx, double maxDy, double maxDTheta)
        {
            return parameterizeSpline(s, maxDx, maxDy, maxDTheta, 0.0, 1.0);
        }

        public static List<Pose2dWithCurvature> parameterizeSplines(List<QuinticSpline> splines)
        {
            return parameterizeSplines(splines, kMaxDX, kMaxDY, kMaxDTheta);
        }

        public static List<Pose2dWithCurvature> parameterizeSplines(List<QuinticSpline> splines, double maxDx,
                double maxDy, double maxDTheta)
        {
            List<Pose2dWithCurvature> rv = new List<Pose2dWithCurvature>();
            if (splines.Count == 0)
                return rv;
            rv.Add(splines[0].getPose2dWithCurvature(0.0));
            foreach (Spline s in splines)
            {
                List<Pose2dWithCurvature> samples = parameterizeSpline(s, maxDx, maxDy, maxDTheta);
                samples.RemoveAt(0);
                rv.AddRange(samples);
            }
            return rv;
        }

        private static void getSegmentArc(Spline s, ref List<Pose2dWithCurvature> rv, double t0, double t1, double maxDx,
            double maxDy, double maxDTheta)
        {
            Translation2d p0 = s.getPoint(t0);
            Translation2d p1 = s.getPoint(t1);
            Rotation2d r0 = s.getHeading(t0);
            Rotation2d r1 = s.getHeading(t1);
            Pose2d transformation = new Pose2d(new Translation2d(p0, p1).rotateBy(r0.inverse()), r1.rotateBy(r0.inverse()));
            Twist2d twist = Pose2d.log(transformation);
            if (twist.dy > maxDy || twist.dx > maxDx || twist.dtheta > maxDTheta)
            {
                getSegmentArc(s, ref rv, t0, (t0 + t1) / 2, maxDx, maxDy, maxDTheta);
                getSegmentArc(s, ref rv, (t0 + t1) / 2, t1, maxDx, maxDy, maxDTheta);
            }
            else
            {
                rv.Add(s.getPose2dWithCurvature(t1));
            }
        }


    }
}
