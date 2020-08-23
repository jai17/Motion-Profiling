using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

using GeometryLibraries;

namespace ProfilingLibraries
{
    public class UntimedTrajectory
    {
        public struct TrajectoryPoint
        {
            public Pose2dWithCurvature mState;
            public int mIndex;

            public TrajectoryPoint(Pose2dWithCurvature state, int index)
            {
                mState = state;
                mIndex = index;
            }
        }

        public struct TrajectorySamplePoint
        {
            public Pose2dWithCurvature mState;
            public int mIndex_floor;
            public int mIndex_ceil;

            public TrajectorySamplePoint(TrajectoryPoint point)
            {
                mState = point.mState;
                mIndex_floor = mIndex_ceil = point.mIndex;
            }

            public TrajectorySamplePoint(Pose2dWithCurvature state, int index_floor, int index_ceil)
            {
                mState = state;
                mIndex_floor = index_floor;
                mIndex_ceil = index_ceil;
            }
        }

        private List<TrajectoryPoint> points;

        private double default_velocity = 0.0;

        public UntimedTrajectory()
        {
            points = new List<TrajectoryPoint>();
        }

        public UntimedTrajectory(List<Pose2dWithCurvature> states)
        {
            points = new List<TrajectoryPoint>(states.Count);
            for(int i = 0; i < states.Count; i++)
            {
                points.Add(new TrajectoryPoint(states[i], i));
            }
        }

        public void setDefaultVelocity(double default_velocity)
        {
            this.default_velocity = default_velocity;
        }
        public double defaultVelocity()
        {
            return default_velocity;
        }

        public bool isEmpty()
        {
            return points.Count == 0;
        }

        public int length()
        {
            return points.Count;
        }

        public TrajectoryPoint getPoint(int index)
        {
            return points[index];
        }

        public Pose2dWithCurvature getState(int index)
        {
            return points[index].mState;
        }

        public Pose2dWithCurvature getFirstState()
        {
            return getState(0);
        }

        public Pose2dWithCurvature getLastState()
        {
            return getState(length() - 1);
        }

        public TrajectorySamplePoint getInterpolated(double index)
        {
            if (isEmpty())
            {
                return new TrajectorySamplePoint();
            }
            else if (index <= 0.0)
            {
                return new TrajectorySamplePoint(getPoint(0));
            }
            else if (index >= length() - 1)
            {
                return new TrajectorySamplePoint(getPoint(length() - 1));
            }
            int i = (int)Math.Floor(index);
            double frac = index - i;
            if (frac <= Double.MinValue)
            {
                return new TrajectorySamplePoint(getPoint(i));
            }
            else if (frac >= 1.0 - Double.MinValue)
            {
                return new TrajectorySamplePoint(getPoint(i + 1));
            }
            else
            {
                return new TrajectorySamplePoint(getState(i).interpolate(getState(i + 1), frac), i, i + 1);
            }
        }

        public virtual TrajectorySamplePoint sample(double index)
        {
            return this.getInterpolated(index);
        }

        public virtual double last_interpolant()
        {
            return Math.Max(0.0, this.length() - 1);
        }

        public virtual double first_interpolant()
        {
            return 0.0;
        }

        public virtual UntimedTrajectory trajectory() { return this; }
    }


}
