using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace GeometryLibraries
{
    public class Rotation2d
    {
        protected static Rotation2d kIdentity = new Rotation2d();

        public static Rotation2d identity()
        {
            return kIdentity;
        }

        protected double cos_angle_;
        protected double sin_angle_;
        protected double theta_degrees = 0;
        protected double theta_radians = 0;

        private const double kEpsilon = 1e-12;

        public Rotation2d() : this(1, 0, false)
        {
        }

        public Rotation2d(double x, double y, bool normalize)
        {
            if (normalize)
            {
                // From trig, we know that sin^2 + cos^2 == 1, but as we do math on this object
                // we might accumulate rounding errors.
                // Normalizing forces us to re-scale the sin and cos to reset rounding errors.
                double magnitude = Math.Sqrt(Math.Pow(x, 2) + Math.Pow(y, 2));
                if (magnitude > kEpsilon)
                {
                    sin_angle_ = y / magnitude;
                    cos_angle_ = x / magnitude;
                }
                else
                {
                    sin_angle_ = 0;
                    cos_angle_ = 1;
                }
            }
            else
            {
                cos_angle_ = x;
                sin_angle_ = y;
            }
            theta_degrees = Math.Atan2(sin_angle_, cos_angle_) * (180/Math.PI);
        }

        public Rotation2d(Rotation2d other)
        {
            cos_angle_ = other.cos_angle_;
            sin_angle_ = other.sin_angle_;
            theta_degrees = Math.Atan2(sin_angle_, cos_angle_) * (180 / Math.PI);
        }

        public Rotation2d(double theta_degrees)
        {
            cos_angle_ = Math.Cos(theta_degrees * (Math.PI / 180.0));
            sin_angle_ = Math.Sin(theta_degrees * (Math.PI / 180.0));
            this.theta_degrees = theta_degrees;
        }

        public Rotation2d(Translation2d direction, bool normalize) : this(direction.x(), direction.y(), normalize)
        {
        }

        public static Rotation2d fromRadians(double angle_radians)
        {
            return new Rotation2d(Math.Cos(angle_radians), Math.Sin(angle_radians), false);
        }

        public static Rotation2d fromDegrees(double angle_degrees)
        {
            return new Rotation2d(angle_degrees);
        }

        public double cos()
        {
            return cos_angle_;
        }

        public double sin()
        {
            return sin_angle_;
        }

        public double tan()
        {
            if (Math.Abs(cos_angle_) < kEpsilon)
            {
                if (sin_angle_ >= 0.0)
                {
                    return Double.PositiveInfinity;
                }
                else
                {
                    return Double.NegativeInfinity;
                }
            }
            return sin_angle_ / cos_angle_;
        }

        public double getRadians()
        {
            return Math.Atan2(sin_angle_, cos_angle_);
        }

        public double getDegrees()
        {
            return getRadians() * (180 / Math.PI);
        }

        public double getUnboundedDegrees()
        {
            return theta_degrees;
        }

        /**
         * We can rotate this Rotation2d by adding together the effects of it and
         * another rotation.
         *
         * @param other The other rotation. See:
         *              https://en.wikipedia.org/wiki/Rotation_matrix
         * @return This rotation rotated by other.
         */
        public Rotation2d rotateBy(Rotation2d other)
        {
            return new Rotation2d(cos_angle_ * other.cos_angle_ - sin_angle_ * other.sin_angle_,
                    cos_angle_ * other.sin_angle_ + sin_angle_ * other.cos_angle_, true);
        }

        public Rotation2d normal()
        {
            return new Rotation2d(-sin_angle_, cos_angle_, false);
        }

        /**
         * The inverse of a Rotation2d "undoes" the effect of this rotation.
         *
         * @return The opposite of this rotation.
         */
        public Rotation2d inverse()
        {
            return new Rotation2d(cos_angle_, -sin_angle_, false);
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

        public bool isParallel(Rotation2d other)
        {
            return epsilonEquals(Translation2d.cross(toTranslation(), other.toTranslation()), 0.0);
        }

        public Translation2d toTranslation()
        {
            return new Translation2d(cos_angle_, sin_angle_);
        }

        /**
         * @return The pole nearest to this rotation.
         */
        public Rotation2d nearestPole()
        {
            double pole_sin = 0.0;
            double pole_cos = 0.0;
            if (Math.Abs(cos_angle_) > Math.Abs(sin_angle_))
            {
                pole_cos = Math.Sign(cos_angle_);
                pole_sin = 0.0;
            }
            else
            {
                pole_cos = 0.0;
                pole_sin = Math.Sign(sin_angle_);
            }
            return new Rotation2d(pole_cos, pole_sin, false);
        }

        public Rotation2d interpolate(Rotation2d other, double x)
        {
            if (x <= 0)
            {
                return new Rotation2d(this);
            }
            else if (x >= 1)
            {
                return new Rotation2d(other);
            }
            double angle_diff = inverse().rotateBy(other).getRadians();
            return this.rotateBy(Rotation2d.fromRadians(angle_diff * x));
        }


        public double distance(Rotation2d other)
        {
            return inverse().rotateBy(other).getRadians();
        }

        public Rotation2d getRotation()
        {
            return this;
        }
    }
}
