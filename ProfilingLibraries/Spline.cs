﻿using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

using GeometryLibraries;

namespace ProfilingLibraries
{
    public abstract class Spline
    {

        public abstract Translation2d getPoint(double t);

        public abstract Rotation2d getHeading(double t);

        public abstract double getCurvature(double t);

        // dk/ds
        public abstract double getDCurvature(double t);

        // ds/dt
        public abstract double getVelocity(double t);

        public Pose2d getPose2d(double t)
        {
            return new Pose2d(getPoint(t), getHeading(t));
        }

        public Pose2dWithCurvature getPose2dWithCurvature(double t)
        {
            return new Pose2dWithCurvature(getPose2d(t), getCurvature(t), getDCurvature(t) / getVelocity(t));
        }
    }
}
