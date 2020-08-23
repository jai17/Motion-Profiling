using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace ProfilingLibraries
{
    public class UntimedTrajectoryDistances : UntimedTrajectory
    {

        private UntimedTrajectory mTrajectory;
        protected double[] distances;

        public const double kEpsilon = 1e-12;

        public UntimedTrajectoryDistances(UntimedTrajectory trajectory)
        {
            mTrajectory = trajectory;
            distances = new double[trajectory.length()];
            distances[0] = 0.0;
            for (int i = 1; i < trajectory.length(); ++i)
            {
                distances[i] = distances[i - 1] + trajectory.getState(i - 1).distance(trajectory.getState(i));
            }
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

        override
        public TrajectorySamplePoint sample(double distance)
        {
            if (distance >= last_interpolant())
                return new TrajectorySamplePoint(mTrajectory.getPoint(mTrajectory.length() - 1));
            if (distance <= 0.0)
                return new TrajectorySamplePoint(mTrajectory.getPoint(0));
            for (int i = 1; i < distances.Length; ++i)
            {
                TrajectoryPoint s = mTrajectory.getPoint(i);
                if (distances[i] >= distance)
                {
                    TrajectoryPoint prev_s = mTrajectory.getPoint(i - 1);
                    if (epsilonEquals(distances[i], distances[i - 1]))
                    {
                        return new TrajectorySamplePoint(s);
                    }
                    else
                    {
                        return new TrajectorySamplePoint(prev_s.mState.interpolate(s.mState,
                                (distance - distances[i - 1]) / (distances[i] - distances[i - 1])), i - 1, i);
                    }
                }
            }
            throw new Exception();
        }

        override
        public double last_interpolant()
        {
            return distances[distances.Length - 1];
        }

        override
        public double first_interpolant()
        {
            return 0.0;
        }

        override
        public UntimedTrajectory trajectory()
        {
            return mTrajectory;
        }
    }
}
