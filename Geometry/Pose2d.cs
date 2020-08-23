using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace GeometryLibraries
{
    public class Pose2d
    {
        protected static Pose2d kIdentity = new Pose2d();

        public const double kEpsilon = 1e-12;

        public static Pose2d identity()
        {
            return kIdentity;
        }

        private static double kEps = 1E-9;

        protected Translation2d translation_;
        protected Rotation2d rotation_;

        public Pose2d()
        {
            translation_ = new Translation2d();
            rotation_ = new Rotation2d();
        }

        public Pose2d(double x, double y, Rotation2d rotation)
        {
            translation_ = new Translation2d(x, y);
            rotation_ = rotation;
        }

        public Pose2d(Translation2d translation, Rotation2d rotation)
        {
            translation_ = translation;
            rotation_ = rotation;
        }

        public Pose2d(Pose2d other)
        {
            translation_ = new Translation2d(other.translation_);
            rotation_ = new Rotation2d(other.rotation_);
        }

        public static Pose2d fromTranslation(Translation2d translation)
        {
            return new Pose2d(translation, new Rotation2d());
        }

        public static Pose2d fromRotation(Rotation2d rotation)
        {
            return new Pose2d(new Translation2d(), rotation);
        }

        /**
         * Obtain a new Pose2d from a (constant curvature) velocity. See:
         * https://github.com/strasdat/Sophus/blob/master/sophus/se2.hpp
         */
        public static Pose2d exp(Twist2d delta)
        {
            double sin_theta = Math.Sin(delta.dtheta);
            double cos_theta = Math.Cos(delta.dtheta);
            double s, c;
            if (Math.Abs(delta.dtheta) < kEps)
            {
                s = 1.0 - 1.0 / 6.0 * delta.dtheta * delta.dtheta;
                c = .5 * delta.dtheta;
            }
            else
            {
                s = sin_theta / delta.dtheta;
                c = (1.0 - cos_theta) / delta.dtheta;
            }
            return new Pose2d(new Translation2d(delta.dx * s - delta.dy * c, delta.dx * c + delta.dy * s),
                    new Rotation2d(cos_theta, sin_theta, false));
        }

        /**
         * Logical inverse of the above.
         */
        public static Twist2d log(Pose2d transform)
        {
            double dtheta = transform.getRotation().getRadians();
            double half_dtheta = 0.5 * dtheta;
            double cos_minus_one = transform.getRotation().cos() - 1.0;
            double halftheta_by_tan_of_halfdtheta;
            if (Math.Abs(cos_minus_one) < kEps)
            {
                halftheta_by_tan_of_halfdtheta = 1.0 - 1.0 / 12.0 * dtheta * dtheta;
            }
            else
            {
                halftheta_by_tan_of_halfdtheta = -(half_dtheta * transform.getRotation().sin()) / cos_minus_one;
            }
            Translation2d translation_part = transform.getTranslation()
                    .rotateBy(new Rotation2d(halftheta_by_tan_of_halfdtheta, -half_dtheta, false));
            return new Twist2d(translation_part.x(), translation_part.y(), dtheta);
        }

        public Translation2d getTranslation()
        {
            return translation_;
        }

        public Rotation2d getRotation()
        {
            return rotation_;
        }

        /**
         * Transforming this RigidTransform2d means first translating by
         * other.translation and then rotating by other.rotation
         *
         * @param other The other transform.
         * @return This transform * other
         */
        public Pose2d transformBy(Pose2d other)
        {
            return new Pose2d(translation_.translateBy(other.translation_.rotateBy(rotation_)),
                    rotation_.rotateBy(other.rotation_));
        }

        /**
         * The inverse of this transform "undoes" the effect of translating by this
         * transform.
         *
         * @return The opposite of this transform.
         */
        public Pose2d inverse()
        {
            Rotation2d rotation_inverted = rotation_.inverse();
            return new Pose2d(translation_.inverse().rotateBy(rotation_inverted), rotation_inverted);
        }

        public Pose2d normal()
        {
            return new Pose2d(translation_, rotation_.normal());
        }

        private static Translation2d intersectionInternal(Pose2d a, Pose2d b)
        {
            Rotation2d a_r = a.getRotation();
            Rotation2d b_r = b.getRotation();
            Translation2d a_t = a.getTranslation();
            Translation2d b_t = b.getTranslation();

            double tan_b = b_r.tan();
            double t = ((a_t.x() - b_t.x()) * tan_b + b_t.y() - a_t.y()) / (a_r.sin() - a_r.cos() * tan_b);
            if (Double.IsNaN(t))
            {
                return new Translation2d(Double.PositiveInfinity, Double.PositiveInfinity);
            }
            return a_t.translateBy(a_r.toTranslation().scale(t));
        }

        public static bool epsilonEquals(double a, double b, double epsilon)
        {
            return (a - epsilon <= b) && (a + epsilon >= b);
        }

        public static bool epsilonEquals(double a, double b)
        {
            return epsilonEquals(a, b, kEpsilon);
        }

        public static bool epsilonEquals(int a, int b, int epsilon)
        {
            return (a - epsilon <= b) && (a + epsilon >= b);
        }

        /**
         * Return true if this pose is (nearly) colinear with the another.
         */
        public bool isColinear(Pose2d other)
        {
            if (!getRotation().isParallel(other.getRotation()))
                return false;
            Twist2d twist = log(inverse().transformBy(other));
            return (epsilonEquals(twist.dy, 0.0) && epsilonEquals(twist.dtheta, 0.0));
        }

        /**
         * Do twist interpolation of this pose assuming constant curvature.
         */
        public Pose2d interpolate(Pose2d other, double x)
        {
            if (x <= 0)
            {
                return new Pose2d(this);
            }
            else if (x >= 1)
            {
                return new Pose2d(other);
            }
            Twist2d twist = Pose2d.log(inverse().transformBy(other));
            return transformBy(Pose2d.exp(twist.scaled(x)));
        }

        public double distance(Pose2d other)
        {
            return Pose2d.log(inverse().transformBy(other)).norm();
        }


        public Pose2d getPose()
        {
            return this;
        }

        public Pose2d mirror()
        {
            return new Pose2d(new Translation2d(getTranslation().x(), -getTranslation().y()), getRotation().inverse());
        }


    }
}
