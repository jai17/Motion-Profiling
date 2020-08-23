using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace GeometryLibraries
{
    public class Pose2dWithCurvature
    {
        protected static Pose2dWithCurvature kIdentity = new Pose2dWithCurvature();

        public static Pose2dWithCurvature identity()
        {
            return kIdentity;
        }

        protected  Pose2d pose_;
        protected  double curvature_;
        protected  double dcurvature_ds_;

        public Pose2dWithCurvature()
        {
            pose_ = new Pose2d();
            curvature_ = 0.0;
            dcurvature_ds_ = 0.0;
        }

        public Pose2dWithCurvature(Pose2d pose, double curvature)
        {
            pose_ = pose;
            curvature_ = curvature;
            dcurvature_ds_ = 0.0;
        }

        public Pose2dWithCurvature(Pose2d pose, double curvature, double dcurvature_ds)
        {
            pose_ = pose;
            curvature_ = curvature;
            dcurvature_ds_ = dcurvature_ds;
        }

        public Pose2dWithCurvature(Translation2d translation, Rotation2d rotation, double curvature)
        {
            pose_ = new Pose2d(translation, rotation);
            curvature_ = curvature;
            dcurvature_ds_ = 0.0;
        }

        public Pose2dWithCurvature(Translation2d translation, Rotation2d rotation, double curvature,
                double dcurvature_ds)
        {
            pose_ = new Pose2d(translation, rotation);
            curvature_ = curvature;
            dcurvature_ds_ = dcurvature_ds;
        }

        public Pose2d getPose()
        {
            return pose_;
        }

        public Pose2dWithCurvature transformBy(Pose2d transform)
        {
            return new Pose2dWithCurvature(getPose().transformBy(transform), getCurvature(), getDCurvatureDs());
        }

        public Pose2dWithCurvature mirror()
        {
            return new Pose2dWithCurvature(getPose().mirror().getPose(), -getCurvature(), -getDCurvatureDs());
        }

        public double getCurvature()
        {
            return curvature_;
        }

        public double getDCurvatureDs()
        {
            return dcurvature_ds_;
        }

        public Translation2d getTranslation()
        {
            return getPose().getTranslation();
        }

        public Rotation2d getRotation()
        {
            return getPose().getRotation();
        }

        /**
        * Limits the given input to the given magnitude.
        */

        public static double limit(double v, double maxMagnitude)
        {
            return limit(v, -maxMagnitude, maxMagnitude);
        }

        public static double limit(double v, double min, double max)
        {
            return Math.Min(max, Math.Max(min, v));
        }

        public static double interpolate(double a, double b, double x)
        {
            x = limit(x, 0.0, 1.0);
            return a + (b - a) * x;
        }

        public Pose2dWithCurvature interpolate(Pose2dWithCurvature other, double x)
        {
            return new Pose2dWithCurvature(getPose().interpolate(other.getPose(), x),
                    interpolate(getCurvature(), other.getCurvature(), x),
                    interpolate(getDCurvatureDs(), other.getDCurvatureDs(), x));
        }

       public double distance(Pose2dWithCurvature other)
        {
            return getPose().distance(other.getPose());
        }

    }
}
