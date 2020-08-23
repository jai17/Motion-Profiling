using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

using GeometryLibraries;

namespace ProfilingLibraries
{
    public class TrajectoryStatePoint
    {
        private Pose2dWithCurvature mState;
        private double t;
        private double velocity;
        private double acceleration;

        public const double kEpsilon = 1e-12;

        public TrajectoryStatePoint(Pose2dWithCurvature state)
        {
            mState = state;
        }

        public TrajectoryStatePoint(Pose2dWithCurvature state, double t, double velocity, double acceleration)
        {
            mState = state;
            this.t = t;
            this.velocity = velocity;
            this.acceleration = acceleration;
        }

        public Pose2dWithCurvature get_state()
        {
            return mState;
        }

        public void set_t(double t)
        {
            this.t = t;
        }

        public double get_t()
        {
            return t;
        }

        public void set_velocity(double velocity)
        {
            this.velocity = velocity;
        }

        public double get_velocity()
        {
            return velocity;
        }

        public void set_acceleration(double acceleration)
        {
            this.acceleration = acceleration;
        }

        public double get_acceleration()
        {
            return acceleration;
        }

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


        public TrajectoryStatePoint interpolate(TrajectoryStatePoint other, double x)
        {
            double new_t = interpolate(get_t(), other.get_t(), x);
            double delta_t = new_t - get_t();
            if (delta_t < 0.0)
            {
                return other.interpolate(this, 1.0 - x);
            }
            bool reversing = get_velocity() < 0.0 || (epsilonEquals(get_velocity(), 0.0) && get_acceleration() < 0.0);
            double new_v = get_velocity() + get_acceleration() * delta_t;
            double new_s = (reversing ? -1.0 : 1.0) * (get_velocity() * delta_t + .5 * get_acceleration() * delta_t * delta_t);
            // System.out.println("x: " + x + " , new_t: " + new_t + ", new_s: " + new_s + " , distance: " + state()
            // .distance(other.state()));
            return new TrajectoryStatePoint(get_state().interpolate(other.get_state(), new_s / get_state().distance(other.get_state())),
                    new_t,
                    new_v,
                    get_acceleration());
        }

        public double distance(TrajectoryStatePoint other)
        {
            return get_state().distance(other.get_state());
        }
    }
}
