using System;
using System.Globalization;
using System.Reflection;
using System.Reflection.Metadata;
using System.Security.Permissions;
using System.Text;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Data;
using System.Windows.Documents;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Media.Imaging;
using System.Windows.Media.Media3D;
using System.Windows.Navigation;
using System.Windows.Shapes;
using static System.Net.Mime.MediaTypeNames;

namespace _2DRobot_Inverse
{
    /// <summary>
    /// Interaction logic for MainWindow.xaml
    /// </summary>
    public partial class MainWindow : Window
    {
        System.Windows.Threading.DispatcherTimer timer;
        DrawingVisual visual;
        DrawingContext dc;
        double width, height;
        Robot robot;
        Axis axis;
        Point mouse;

        public MainWindow()
        {
            InitializeComponent();
            Init();
        }

        void Init()
        {
            width = g.Width;
            height = g.Height;

            axis = new Axis(width, height);
            robot = new Robot(width, height);

            sljoint1.Value = 0;
            sljoint2.Value = 0;

            visual = new DrawingVisual();

            timer = new System.Windows.Threading.DispatcherTimer();
            timer.Tick += new EventHandler(timerTick);
            timer.Interval = new TimeSpan(0, 0, 0, 0, 100);

            timer.Start();
        }

        private void timerTick(object sender, EventArgs e)
        {
            g.RemoveVisual(visual);
            using (dc = visual.RenderOpen())
            {
                // axis drawing
                axis.Draw(dc, visual);

                // Joints drawing
                Joint next = robot.joint_1;
                while (next != null)
                {
                    next.Update();
                    next.Draw(dc);
                    next = next.child;
                }

                // Tool drawing
                //Point toolPos = robot.joint_2.GetEndPos();
                Point toolPos = FwdCalculation();

                dc.DrawEllipse(Brushes.Red, null, toolPos, 3, 3);

                // Tool coordinates drawing
                var toolText = "(" + axis.ToX(Math.Round(toolPos.X)) + ", " + axis.ToY(Math.Round(toolPos.Y)) + ")";
                FormattedText toolFormattedText = new FormattedText(toolText, CultureInfo.GetCultureInfo("en-us"),
                                                                FlowDirection.LeftToRight, new Typeface("Verdana"), 12, Brushes.White,
                                                                VisualTreeHelper.GetDpi(visual).PixelsPerDip);
                dc.DrawText(toolFormattedText, toolPos);

                // Base drawing
                double rect = 5;
                dc.DrawRectangle(Brushes.Orange, null, new Rect(robot.joint_1.GetStartPos().X - rect/2, robot.joint_1.GetStartPos().Y - rect, rect, rect * 2));

                // Mouse position drawing
                string text = "";
                double length = robot.GetJointsLength();
                double mouseDist = Numerics.GetDistance(new Point(0, 0), new Point(axis.ToX(mouse.X), axis.ToY(mouse.Y)));
                if (mouseDist < length)
                    text = "(" + axis.ToX(Math.Round(mouse.X)) + ", " + axis.ToY(Math.Round(mouse.Y)) + ")";
                else
                    text = "Not reacheble";
                FormattedText formattedText = new FormattedText(text, CultureInfo.GetCultureInfo("en-us"),
                                                FlowDirection.LeftToRight, new Typeface("Verdana"), 12, Brushes.Yellow,
                                                VisualTreeHelper.GetDpi(visual).PixelsPerDip);
                dc.DrawText(formattedText, new Point(mouse.X, mouse.Y - 15));

                // 

                dc.Close();
                g.AddVisual(visual);
            }
        }

        Point FwdCalculation()
        {
            Point p = new Point();

            var L1 = robot.joint_1.GetLen();
            var Q1 = robot.joint_1.GetRadAngle() + -Math.PI / 2;

            var L2 = robot.joint_2.GetLen();
            var Q2 = robot.joint_2.GetRadAngle();

            // Formula:
            // x = XA + x' = L1*cos(Q1) + L2*cos(Q1+Q2)
            // y = YA + y' = L1*sin(Q1) + L2*sin(Q1+Q2)
            var x = robot.joint_1.GetStartPos().X + L1 * Math.Cos(Q1) + L2 * Math.Cos(Q1 + Q2);
            var y = robot.joint_1.GetStartPos().Y + L1 * Math.Sin(Q1) + L2 * Math.Sin(Q1 + Q2);

            p.X = Math.Round(x);
            p.Y = Math.Round(y);

            return p;
        }

        /// <summary>
        /// Basen On:
        /// http://robocraft.ru/blog/mechanics/756.html
        /// </summary>
        /// <param name="target"></param>
        void InverseCalculation(Point target)
        {
            var x = target.X - robot.joint_1.GetStartPos().X;
            var y = target.Y - robot.joint_1.GetStartPos().Y;

            // B - воображаемая линия
            var B = Math.Sqrt(x * x + y * y);

            // q1 — угол между осью OX и прямой B
            var q1 = Math.Atan2(y, x);

            var L1 = robot.joint_1.GetLen();
            var L2 = robot.joint_2.GetLen();

            // q2 — угол между прямой B и плечом L1
            var d = (L2 * L2 + B * B - L1 * L1) / (2 * B * L1);
            var q2 = Math.Acos(d);

            // Q1 = q1 - q2
            var Q1 = q1 - q2 + Math.PI / 2;

            var Q2 = Math.PI - Math.Acos((L1 * L1 + L2 * L2 - B * B) / (2 * L1 * L2));

            robot.joint_1.SetAngle(Q1);
            robot.joint_2.SetAngle(Q2);
        }

        private void sljoint1_ValueChanged(object sender, RoutedPropertyChangedEventArgs<double> e)
        {
            robot.joint_1.SetAngle(Numerics.ToRadians(sljoint1.Value));
            lbJ1Angle.Content = robot.joint_1.GetDegAngle();
        }
        private void sljoint2_ValueChanged(object sender, RoutedPropertyChangedEventArgs<double> e)
        {
            robot.joint_2.SetAngle(Numerics.ToRadians(sljoint2.Value));
            lbJ2Angle.Content = robot.joint_2.GetDegAngle();
        }
        
        private void g_MouseMove(object sender, MouseEventArgs e)
        {
            mouse = e.GetPosition(g);

            double length = robot.GetJointsLength();
            double mouseDist = Numerics.GetDistance(new Point(0, 0), new Point(axis.ToX(mouse.X), axis.ToY(mouse.Y)));
            if (mouseDist < length)
                InverseCalculation(mouse);
        }
    }

    class Robot
    {
        public Joint joint_1, joint_2;


        public Robot(double w, double h)
        {
            joint_1 = new Joint(w / 2, h / 2, 50, 0);
            joint_2 = new Joint(joint_1, 50, 0);
            joint_1.child = joint_2;
        }

        public double GetJointsLength()
        {
            return joint_1.GetLen() + joint_2.GetLen();
        }

        public void Draw()
        {

        }
    }

    class Joint
    {
        Point pStart, pEnd;
        public double angle, selfAngle, len;
        Joint parent;
        Brush color = Brushes.White;
        public Joint child;

        public Joint(double x, double y, double length, double angle)
        {
            pStart = new Point(x, y);
            len = length;
            selfAngle = Numerics.ToRadians(angle);
            parent = null;
            CalculateEndPoint();
        }

        public Joint(Joint parent, double length, double angle)
        {
            this.parent = parent;
            pStart = parent.pEnd;
            len = length;
            selfAngle = Numerics.ToRadians(angle);
            CalculateEndPoint();
        }

        public void SetAngle(double value) => selfAngle = value;
        public double GetDegAngle() => Math.Round(Numerics.ToDegrees(selfAngle));
        public double GetRadAngle() => selfAngle;
        public Point GetStartPos() => pStart;
        public Point GetEndPos() => pEnd;
        public double GetLen() => len;
        public void Update()
        {
            angle = selfAngle;
            if (parent != null)
            {
                pStart = parent.pEnd;
                angle += parent.angle;
            }
            else
            {
                // to turn axis 90 degrees
                angle += -Math.PI / 2;
            }
            CalculateEndPoint();
        }
        void CalculateEndPoint()
        {
            double dx = len * Math.Cos(angle);
            double dy = len * Math.Sin(angle);
            pEnd = new Point(pStart.X + dx, pStart.Y + dy);
        }
        public void Draw(DrawingContext dc) => dc.DrawLine(new Pen(color, 2), pStart, pEnd);
    }

    static class Numerics
    {
        public static double ToRadians(double num) => num * Math.PI / 180.0;
        public static double ToDegrees(double num) => num * 180.0 / Math.PI;
        public static double GetDistance(Point a, Point b) => Math.Sqrt((a.X - b.X)* (a.X - b.X) + (a.Y - b.Y) * (a.Y - b.Y));
    }
}