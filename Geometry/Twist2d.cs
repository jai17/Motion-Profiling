using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace GeometryLibraries
{
    public class Twist2d
    {
        protected static Twist2d kIdentity = new Twist2d(0.0, 0.0, 0.0);

        private const double kEpsilon = 1e-12;

        public static Twist2d identity()
        {
            return kIdentity;
        }

        public double dx;
        public double dy;
        public double dtheta; // Radians!

        public Twist2d(double dx, double dy, double dtheta)
        {
            this.dx = dx;
            this.dy = dy;
            this.dtheta = dtheta;
        }

        public Twist2d scaled(double scale)
        {
            return new Twist2d(dx * scale, dy * scale, dtheta * scale);
        }

        public double norm()
        {
            // Common case of dy == 0
            if (dy == 0.0)
                return Math.Abs(dx);
            return Math.Sqrt(Math.Pow(dx,2) + Math.Pow(dy,2));
        }

        public double curvature()
        {
            if (Math.Abs(dtheta) < kEpsilon && norm() < kEpsilon)
                return 0.0;
            return dtheta / norm();
        }

    }
}
