using System;
using System.Collections.Generic;
using System.Globalization;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Media;
using System.Windows;

namespace _2DRobot_Inverse
{
    internal class Axis
    {
        double width, height;
        Pen pen;

        public Axis(double w, double h)
        {
            width = w;
            height = h;

            pen = new Pen(Brushes.White, 1);
        }

        public void Draw(DrawingContext dc, DrawingVisual visual)
        {
            // axis X
            dc.DrawLine(pen, new Point(0, height / 2), new Point(width, height / 2));

            // axis Y
            dc.DrawLine(pen, new Point(width / 2, 0), new Point(width / 2, height));
        }

        public double ToX(double x) => x - width / 2;
        public double ToY(double y) => (y - height / 2) * -1;
    }
}
