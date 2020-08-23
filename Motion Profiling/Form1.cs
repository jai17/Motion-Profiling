using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Drawing;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Forms;

using ProfilingLibraries;
using GeometryLibraries;

namespace Motion_Profiling
{
    public partial class Form1 : Form
    {
        public Form1()
        {
            InitializeComponent();

            //Create a list of waypoints
            List<Pose2d> waypoints = new List<Pose2d>(3);
            waypoints.Add(new Pose2d(new Translation2d(0, 0), Rotation2d.fromDegrees(0)));
            waypoints.Add(new Pose2d(new Translation2d(100, 20), Rotation2d.fromDegrees(0)));
            waypoints.Add(new Pose2d(new Translation2d(254.8, 50.87), Rotation2d.fromDegrees(45)));

            // For a reversed trajectory
            List<Pose2d> waypoints_maybe_flipped = waypoints;
            Pose2d flip = Pose2d.fromRotation(new Rotation2d(-1, 0, false));
            if (false)
            {
                waypoints_maybe_flipped = new List<Pose2d>(waypoints.Count);
                for (int i = 0; i < waypoints.Count; ++i)
                {
                    waypoints_maybe_flipped.Add(waypoints[i].transformBy(flip));
                }
            }

            //Create a list of splines between each pair of waypoints
            List<QuinticSpline> splines = new List<QuinticSpline>(waypoints.Count - 1);
            for (int i = 1; i < waypoints.Count; ++i)
            {
                splines.Add(new QuinticSpline(waypoints[i - 1], waypoints[i]));
            }

            //Doesnt do much for simple curves
            //Optimize spline based on curvature
            QuinticSpline.optimizeSpline(ref splines);

            Console.WriteLine("Spline Unsegmented");
            foreach (QuinticSpline S in splines)
            {
                for (double t = 0; t <= 1; t += 1 / 100.0)
                {
                    Console.WriteLine(S.getPoint(t).x() + " , " + S.getPoint(t).y() + " , " + S.getHeading(t).getDegrees());
                }
            }

            Console.WriteLine("Spline Combined and Segmented");

            //Create the untimed trajectory (Splines into segment generator)
            UntimedTrajectory trajectory = new UntimedTrajectory(SegmentedSplineGenerator.parameterizeSplines(splines));

            for(int i = 0; i < trajectory.length(); i++)
            {
                double x = trajectory.getState(i).getTranslation().x();
                double y = trajectory.getState(i).getTranslation().y();
                double theta = trajectory.getState(i).getRotation().getDegrees();
                double curvature = trajectory.getState(i).getCurvature();
                double dCurvatureDs = trajectory.getState(i).getDCurvatureDs();

                Console.WriteLine(i + " , " + x + " , " + y + " , " + theta + " , " + curvature + " , " + dCurvatureDs);

            }

            //For a reversed trajectory
            if (false)
            {
                List<Pose2dWithCurvature> flipped = new List<Pose2dWithCurvature>(trajectory.length());
                for (int i = 0; i < trajectory.length(); ++i)
                {
                    flipped.Add(new Pose2dWithCurvature(trajectory.getState(i).getPose().transformBy(flip),
                            -trajectory.getState(i).getCurvature(), trajectory.getState(i).getDCurvatureDs()));
                }
                trajectory = new UntimedTrajectory(flipped);
            }

            Console.WriteLine("Trajectory");

            TrajectoryContainer final_trajectory = TrajectoryGenerator.parameterizeTrajectory(false, trajectory, 2.0, 0.0, 0.0, 120.0, 120.0, 24.0, 20);

            final_trajectory.setDefaultVelocity(72.0 / 150.0);

            for (int i = 0; i < final_trajectory.length(); i++)
            {
                double x = final_trajectory.getState(i).get_state().getTranslation().x();
                double y = final_trajectory.getState(i).get_state().getTranslation().y();
                double position = Math.Sqrt(x*x + y*y);
                double theta = final_trajectory.getState(i).get_state().getRotation().getDegrees();
                double time = final_trajectory.getState(i).get_t();
                double velocity = final_trajectory.getState(i).get_velocity();
                double accel = final_trajectory.getState(i).get_acceleration();
                Console.WriteLine(time + " , " + position + " , " + velocity + " , " + accel);

            }

        }

        private void cartesianChart1_ChildChanged(object sender, System.Windows.Forms.Integration.ChildChangedEventArgs e)
        {

        }
    }
}
