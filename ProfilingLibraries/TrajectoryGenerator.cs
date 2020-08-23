using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

using GeometryLibraries;

namespace ProfilingLibraries
{
    public class TrajectoryGenerator
    {
        protected struct ConstrainedState {
            public Pose2dWithCurvature state;
            public double distance;
            public double max_velocity;
            public double min_acceleration;
            public double max_acceleration;
        }

        public static TrajectoryContainer parameterizeTrajectory(
            bool reverse,
            UntimedTrajectory trajectory,
            double step_size,
            double start_velocity,
            double end_velocity,
            double max_velocity,
            double max_abs_acceleration,
            double max_deceleration,
            int slowdown_chunks)
        {
            UntimedTrajectoryDistances traj_distances = new UntimedTrajectoryDistances(trajectory);

            int num_states = (int)Math.Ceiling(traj_distances.last_interpolant() / step_size + 1);
            List<Pose2dWithCurvature> states = new List<Pose2dWithCurvature>(num_states);
            for (int i = 0; i < num_states; ++i)
            {
                states.Add(traj_distances.sample(Math.Min(i * step_size, traj_distances.last_interpolant())).mState);
            }

            List<ConstrainedState> constrained_states = new List<ConstrainedState>(states.Count);
            const double kEpsilon = 1e-6;

            //Forward Pass
            ConstrainedState previous;
            previous.state = states[0];
            previous.distance = 0.0;
            previous.max_velocity = start_velocity;
            previous.min_acceleration = -max_abs_acceleration;
            previous.max_acceleration = max_abs_acceleration;

            for(int i =  0; i < states.Count; i++)
            {
                //Create the next state
                ConstrainedState current = new ConstrainedState();
                current.state = states[i];
                //Distance is the distance from this state to the previous one
                double ds = current.state.distance(previous.state);
                //Accumulated distance is the new distance plus the accumulating sum
                current.distance = ds + previous.distance;

                while (true)
                {
                    // Change the max velocity to the max reachable velocity by the acceleration limit.
                    // vf = sqrt(vi^2 + 2*a*d)
                    current.max_velocity = Math.Min(max_velocity,
                            Math.Sqrt(previous.max_velocity * previous.max_velocity
                                    + 2.0 * previous.max_acceleration * ds));

                    // Enforce max absolute acceleration.
                    current.min_acceleration = -max_abs_acceleration;
                    current.max_acceleration = max_abs_acceleration;

                    if (ds < kEpsilon)
                    {
                        break;
                    }

                    //If the maximum acceleration is less than the actual acceleration, need to reduce the previous states acceleration and loop again
                    // a = (vf^2 - vi^2) / 2d
                    double actual_acceleration = (current.max_velocity * current.max_velocity
                        - previous.max_velocity * previous.max_velocity) / (2.0 * ds);
                    if (current.max_acceleration < actual_acceleration - kEpsilon)
                    {
                        previous.max_acceleration = current.max_acceleration;
                    }
                    else
                    {
                        if (actual_acceleration > previous.min_acceleration + kEpsilon)
                        {
                            previous.max_acceleration = actual_acceleration;
                        }
                        // If actual acceleration is less than predecessor min accel, we will repair during the backward pass.
                        break;
                    }
                }
                if (i > 0)
                    constrained_states[i - 1] = previous;
                constrained_states.Add(current);
                previous = current;
            }

            //Backward Pass
            ConstrainedState next = new ConstrainedState();
            next.state = states[states.Count - 1];
            next.distance = constrained_states[states.Count - 1].distance;
            next.max_velocity = end_velocity;
            next.min_acceleration = -max_deceleration;
            next.max_acceleration = max_abs_acceleration;

            for(int i = states.Count - 1; i >=0; i--)
            {
                ConstrainedState current = constrained_states[i];
                double ds = current.distance - next.distance;

                //If the state is a slowdown state, apply the max deceleration
                if (i >= states.Count - slowdown_chunks)
                    current.min_acceleration = -max_deceleration;

                while (true)
                {
                    // Enforce reverse max reachable velocity limit.
                    // vf = sqrt(vi^2 + 2*a*d), where vi = next.
                    double new_max_velocity = Math.Sqrt(next.max_velocity * next.max_velocity
                            + 2.0 * next.min_acceleration * ds);
                    if (new_max_velocity >= current.max_velocity)
                    {
                        // No new limits to impose.
                        break;
                    }
                    current.max_velocity = new_max_velocity;
                    if (ds > kEpsilon)
                    {
                        break;
                    }

                    // If the min acceleration for this state is less than the actual, need to reduce the min accel and loop again
                    // a = (vf^2 - vi^2) / 2d
                    double actual_acceleration = (current.max_velocity * current.max_velocity
                            - next.max_velocity * next.max_velocity) / (2.0 * ds);
                    if (current.min_acceleration > actual_acceleration + kEpsilon)
                    {
                        next.min_acceleration = current.min_acceleration;
                    }
                    else
                    {
                        next.min_acceleration = actual_acceleration;
                        break;
                    }
                }
                if (i < states.Count - 1)
                    constrained_states[i + 1] = next;
                constrained_states[i] = current;
                next = current;
            }

            List<TrajectoryStatePoint> final_states = new List<TrajectoryStatePoint>(states.Count);
            double time = 0;
            double distance = 0;
            double velocity = 0;

            for(int i = 0; i < states.Count; i++)
            {
                ConstrainedState current = constrained_states[i];

                double ds = current.distance - distance;
                double accel = (current.max_velocity * current.max_velocity - velocity * velocity) / (2.0 * ds);
                if ((Double.IsNaN(accel) || Math.Abs(accel) <= kEpsilon) && (Double.IsNaN(velocity) || Math.Abs(velocity) <= kEpsilon) && i > 0)
                {
                    Console.WriteLine("PATH GENERATION CORRECTED, WOULD HAVE FAILED OTHERWISE");
                    accel = 0.00001;
                }

                double dt = 0.0;
                if(i > 0)
                {
                    final_states[i - 1].set_acceleration(reverse ? -accel : accel);
                    if (Math.Abs(accel) > kEpsilon)
                        dt = (current.max_velocity - velocity) / accel;
                    else
                        dt = ds / velocity;
                }

                time += dt;
                velocity = current.max_velocity;
                distance = current.distance;

                final_states.Add(new TrajectoryStatePoint(current.state, time, reverse ? -velocity : velocity, reverse ? -accel : accel));
            }

            return new TrajectoryContainer(final_states);
        }

    }
}
