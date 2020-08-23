using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

using GeometryLibraries;

namespace ProfilingLibraries
{
    public class QuinticSpline : Spline
    {

        private const double kEpsilon = 1e-5;
        private const double kStepSize = 1.0;
        private const double kMinDelta = 0.001;
        private const int kSamples = 100;
        private const int kMaxIterations = 100;

        //Point constants
        private double x0, x1, dx0, dx1, ddx0, ddx1, y0, y1, dy0, dy1, ddy0, ddy1;

        //Curve Constants
        private double ax, bx, cx, Dx, ex, fx, ay, by, cy, Dy, ey, fy;

        

        /**
        * @param p0  The starting pose of the spline
        * @param p1  The ending pose of the spline
        */
        public QuinticSpline(Pose2d p0, Pose2d p1)
        {
            double scale = 1.2 * p0.getTranslation().distance(p1.getTranslation());
            x0 = p0.getTranslation().x();
            x1 = p1.getTranslation().x();
            dx0 = p0.getRotation().cos() * scale;
            dx1 = p1.getRotation().cos() * scale;
            ddx0 = 0;
            ddx1 = 0;
            y0 = p0.getTranslation().y();
            y1 = p1.getTranslation().y();
            dy0 = p0.getRotation().sin() * scale;
            dy1 = p1.getRotation().sin() * scale;
            ddy0 = 0;
            ddy1 = 0;

            computeCoefficients();
        }

        /**
        * Used by the curvature optimization function
        */
        private QuinticSpline(double x0, double x1, double dx0, double dx1, double ddx0, double ddx1, double y0,
                double y1, double dy0, double dy1, double ddy0, double ddy1)
        {
            this.x0 = x0;
            this.x1 = x1;
            this.dx0 = dx0;
            this.dx1 = dx1;
            this.ddx0 = ddx0;
            this.ddx1 = ddx1;

            this.y0 = y0;
            this.y1 = y1;
            this.dy0 = dy0;
            this.dy1 = dy1;
            this.ddy0 = ddy0;
            this.ddy1 = ddy1;

            computeCoefficients();
        }

        /**
         * Re-arranges the spline into an at^5 + bt^4 + ... + f form for simpler
         * computations
         */
        private void computeCoefficients()
        {
            ax = -6 * x0 - 3 * dx0 - 0.5 * ddx0 + 0.5 * ddx1 - 3 * dx1 + 6 * x1;
            bx = 15 * x0 + 8 * dx0 + 1.5 * ddx0 - ddx1 + 7 * dx1 - 15 * x1;
            cx = -10 * x0 - 6 * dx0 - 1.5 * ddx0 + 0.5 * ddx1 - 4 * dx1 + 10 * x1;
            Dx = 0.5 * ddx0;
            ex = dx0;
            fx = x0;

            ay = -6 * y0 - 3 * dy0 - 0.5 * ddy0 + 0.5 * ddy1 - 3 * dy1 + 6 * y1;
            by = 15 * y0 + 8 * dy0 + 1.5 * ddy0 - ddy1 + 7 * dy1 - 15 * y1;
            cy = -10 * y0 - 6 * dy0 - 1.5 * ddy0 + 0.5 * ddy1 - 4 * dy1 + 10 * y1;
            Dy = 0.5 * ddy0;
            ey = dy0;
            fy = y0;
        }

        public Pose2d getStartPose()
        {
            return new Pose2d(new Translation2d(x0, y0), new Rotation2d(dx0, dy0, true));
        }

        public Pose2d getEndPose()
        {
            return new Pose2d(new Translation2d(x1, y1), new Rotation2d(dx1, dy1, true));
        }

        /**
         * @param t ranges from 0 to 1
         * @return the point on the spline for that t value
         */
        override
        public Translation2d getPoint(double t)
        {
            double x = ax * Math.Pow(t, 5) + bx * Math.Pow(t, 4) + cx * Math.Pow(t, 3) + Dx * Math.Pow(t, 2) + ex * t + fx;
            double y = ay * Math.Pow(t, 5) + by * Math.Pow(t, 4) + cy * Math.Pow(t, 3) + Dy * Math.Pow(t, 2) + ey * t + fy;
            return new Translation2d(x, y);
        }

        private double dx(double t)
        {
            return 5 * ax * Math.Pow(t, 4) + 4 * bx * Math.Pow(t, 3) + 3 * cx * Math.Pow(t, 2) + 2 * Dx * t + ex;
        }

        private double dy(double t)
        {
            return 5 * ay * Math.Pow(t, 4) + 4 * by * Math.Pow(t, 3) + 3 * cy * Math.Pow(t, 2) + 2 * Dy * t + ey;
        }

        private double ddx(double t)
        {
            return 20 * ax * Math.Pow(t, 3) + 12 * bx * Math.Pow(t, 2) + 6 * cx * t + 2 * Dx;
        }

        private double ddy(double t)
        {
            return 20 * ay * Math.Pow(t, 3) + 12 * by * Math.Pow(t, 2) + 6 * cy * t + 2 * Dy;
        }

        private double dddx(double t)
        {
            return 60 * ax * t * t + 24 * bx * t + 6 * cx;
        }

        private double dddy(double t)
        {
            return 60 * ay * t * t + 24 * by * t + 6 * cy;
        }

        override
        public double getVelocity(double t)
        {
            return Math.Sqrt(Math.Pow(dx(t), 2) + Math.Pow(dy(t), 2));
        }

        override
        public Rotation2d getHeading(double t)
        {
            return new Rotation2d(dx(t), dy(t), true);
        }

        override
        public double getCurvature(double t)
        {
            return (dx(t) * ddy(t) - ddx(t) * dy(t))
                    / ((dx(t) * dx(t) + dy(t) * dy(t)) * Math.Sqrt((dx(t) * dx(t) + dy(t) * dy(t))));
        }

        override
        public double getDCurvature(double t)
        {
            double dx2dy2 = (dx(t) * dx(t) + dy(t) * dy(t));
            double num = (dx(t) * dddy(t) - dddx(t) * dy(t)) * dx2dy2
                    - 3 * (dx(t) * ddy(t) - ddx(t) * dy(t)) * (dx(t) * ddx(t) + dy(t) * ddy(t));
            return num / (dx2dy2 * dx2dy2 * Math.Sqrt(dx2dy2));
        }

        private double dCurvature2(double t)
        {
            double dx2dy2 = (dx(t) * dx(t) + dy(t) * dy(t));
            double num = (dx(t) * dddy(t) - dddx(t) * dy(t)) * dx2dy2
                    - 3 * (dx(t) * ddy(t) - ddx(t) * dy(t)) * (dx(t) * ddx(t) + dy(t) * ddy(t));
            return num * num / (dx2dy2 * dx2dy2 * dx2dy2 * dx2dy2 * dx2dy2);
        }


        /**
         * @return integral of dCurvature^2 over the length of the spline
         */
        private double sumDCurvature2()
        {
            double dt = 1.0 / kSamples;
            double sum = 0;
            for (double t = 0; t < 1.0; t += dt)
            {
                sum += (dt * dCurvature2(t));
            }
            return sum;
        }

        /**
         * @return integral of dCurvature^2 over the length of multiple splines
         */
        public static double sumDCurvature2(List<QuinticSpline> splines)
        {
            double sum = 0;
            foreach (QuinticSpline s in splines)
            {
                sum += s.sumDCurvature2();
            }
            return sum;
        }

        /**
         * Makes optimization code a little more readable
         */
        private struct ControlPoint
        {
            public double ddx, ddy;
        }

        /**
         * Finds the optimal second derivative values for a set of splines to reduce the
         * sum of the change in curvature squared over the path
         *
         * @param splines the list of splines to optimize
         * @return the final sumDCurvature2
         */
        public static double optimizeSpline(ref List<QuinticSpline> splines)
        {
            int count = 0;
            double prev = sumDCurvature2(splines);
            while (count < kMaxIterations)
            {
                runOptimizationIteration(ref splines);
                double current = sumDCurvature2(splines);
                if (prev - current < kMinDelta)
                    return current;
                prev = current;
                count++;
            }
            return prev;
        }

        /**
         * Runs a single optimization iteration
         */
        private static void runOptimizationIteration(ref List<QuinticSpline> splines)
        {
            // can't optimize anything with less than 2 splines
            if (splines.Count <= 1)
            {
                return;
            }

            ControlPoint[] controlPoints = new ControlPoint[splines.Count - 1];
            double magnitude = 0;

            for (int i = 0; i < splines.Count - 1; ++i)
            {
                // don't try to optimize colinear points
                if (splines[i].getStartPose().isColinear(splines[i + 1].getStartPose())
                        || splines[i].getEndPose().isColinear(splines[i + 1].getEndPose()))
                {
                    continue;
                }
                double original = sumDCurvature2(splines);
                QuinticSpline temp, temp1;

                temp = splines[i];
                temp1 = splines[i + 1];
                controlPoints[i] = new ControlPoint(); // holds the gradient at a control point

                // calculate partial derivatives of sumDCurvature2
                splines[i] = new QuinticSpline(temp.x0, temp.x1, temp.dx0, temp.dx1, temp.ddx0,
                        temp.ddx1 + kEpsilon, temp.y0, temp.y1, temp.dy0, temp.dy1, temp.ddy0, temp.ddy1);
                splines[i + 1] =  new QuinticSpline(temp1.x0, temp1.x1, temp1.dx0, temp1.dx1, temp1.ddx0 + kEpsilon,
                        temp1.ddx1, temp1.y0, temp1.y1, temp1.dy0, temp1.dy1, temp1.ddy0, temp1.ddy1);
                controlPoints[i].ddx = (sumDCurvature2(splines) - original) / kEpsilon;
                splines[i] = new QuinticSpline(temp.x0, temp.x1, temp.dx0, temp.dx1, temp.ddx0, temp.ddx1, temp.y0,
                        temp.y1, temp.dy0, temp.dy1, temp.ddy0, temp.ddy1 + kEpsilon);
                splines[i + 1] = new QuinticSpline(temp1.x0, temp1.x1, temp1.dx0, temp1.dx1, temp1.ddx0,
                        temp1.ddx1, temp1.y0, temp1.y1, temp1.dy0, temp1.dy1, temp1.ddy0 + kEpsilon, temp1.ddy1);
                controlPoints[i].ddy = (sumDCurvature2(splines) - original) / kEpsilon;

                splines[i] = temp;
                splines[i + 1] = temp1;
                magnitude += controlPoints[i].ddx * controlPoints[i].ddx + controlPoints[i].ddy * controlPoints[i].ddy;
            }

            magnitude = Math.Sqrt(magnitude);

            // minimize along the direction of the gradient
            // first calculate 3 points along the direction of the gradient
            Translation2d p1, p2, p3;
            p2 = new Translation2d(0, sumDCurvature2(splines)); // middle point is at the current location

            for (int i = 0; i < splines.Count - 1; ++i)
            { // first point is offset from the middle location by -stepSize
                if (splines[i].getStartPose().isColinear(splines[i + 1].getStartPose())
                        || splines[i].getEndPose().isColinear(splines[i + 1].getEndPose()))
                {
                    continue;
                }
                // normalize to step size
                controlPoints[i].ddx *= kStepSize / magnitude;
                controlPoints[i].ddy *= kStepSize / magnitude;

                // move opposite the gradient by step size amount
                splines[i].ddx1 -= controlPoints[i].ddx;
                splines[i].ddy1 -= controlPoints[i].ddy;
                splines[i + 1].ddx0 -= controlPoints[i].ddx;
                splines[i + 1].ddy0 -= controlPoints[i].ddy;

                // recompute the spline's coefficients to account for new second derivatives
                splines[i].computeCoefficients();
                splines[i + 1].computeCoefficients();
            }
            p1 = new Translation2d(-kStepSize, sumDCurvature2(splines));

            for (int i = 0; i < splines.Count - 1; ++i)
            { // last point is offset from the middle location by +stepSize
                if (splines[i].getStartPose().isColinear(splines[i + 1].getStartPose())
                        || splines[i].getEndPose().isColinear(splines[i + 1].getEndPose()))
                {
                    continue;
                }
                // move along the gradient by 2 times the step size amount (to return to
                // original location and move by 1
                // step)
                splines[i].ddx1 += 2 * controlPoints[i].ddx;
                splines[i].ddy1 += 2 * controlPoints[i].ddy;
                splines[i + 1].ddx0 += 2 * controlPoints[i].ddx;
                splines[i + 1].ddy0 += 2 * controlPoints[i].ddy;

                // recompute the spline's coefficients to account for new second derivatives
                splines[i].computeCoefficients();
                splines[i + 1].computeCoefficients();
            }

            p3 = new Translation2d(kStepSize, sumDCurvature2(splines));

            double stepSize = fitParabola(p1, p2, p3); // approximate step size to minimize sumDCurvature2 along the
                                                       // gradient

            for (int i = 0; i < splines.Count - 1; ++i)
            {
                if (splines[i].getStartPose().isColinear(splines[i + 1].getStartPose())
                        || splines[i].getEndPose().isColinear(splines[i + 1].getEndPose()))
                {
                    continue;
                }
                // move by the step size calculated by the parabola fit (+1 to offset for the
                // final transformation to find
                // p3)
                controlPoints[i].ddx *= 1 + stepSize / kStepSize;
                controlPoints[i].ddy *= 1 + stepSize / kStepSize;

                splines[i].ddx1 += controlPoints[i].ddx;
                splines[i].ddy1 += controlPoints[i].ddy;
                splines[i + 1].ddx0 += controlPoints[i].ddx;
                splines[i + 1].ddy0 += controlPoints[i].ddy;

                // recompute the spline's coefficients to account for new second derivatives
                splines[i].computeCoefficients();
                splines[i + 1].computeCoefficients();
            }
        }

        /**
         * fits a parabola to 3 points
         *
         * @return the x coordinate of the vertex of the parabola
         */
        private static double fitParabola(Translation2d p1, Translation2d p2, Translation2d p3)
        {
            double A = (p3.x() * (p2.y() - p1.y()) + p2.x() * (p1.y() - p3.y()) + p1.x() * (p3.y() - p2.y()));
            double B = (p3.x() * p3.x() * (p1.y() - p2.y()) + p2.x() * p2.x() * (p3.y() - p1.y())
                    + p1.x() * p1.x() * (p2.y() - p3.y()));
            return -B / (2 * A);
        }
    }
}
