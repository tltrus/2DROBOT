using System.Globalization;
using System.Text;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Data;
using System.Windows.Documents;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Media.Imaging;
using System.Windows.Navigation;
using System.Windows.Shapes;

namespace _2DRobot_InverseGradient
{
    /// <summary>
    /// https://www.alanzucconi.com/2017/04/10/robotic-arms/
    /// </summary>
    public partial class MainWindow : Window
    {
        System.Windows.Threading.DispatcherTimer timer, timer1;
        DrawingVisual visual;
        DrawingContext dc;
        double width, height;
        Robot robot;
        Axis axis;
        Point mouse;
        Point target;
        double LearningRate = 0.002;
        double SamplingDistance = 0.07;
        double DistanceThreshold = 5;
        int movements = 20;

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

            visual = new DrawingVisual();

            timer = new System.Windows.Threading.DispatcherTimer();
            timer.Tick += new EventHandler(timerTick);
            timer.Interval = new TimeSpan(0, 0, 0, 0, 100);

            timer.Start();

            timer1 = new System.Windows.Threading.DispatcherTimer();
            timer1.Tick += new EventHandler(timer1_Tick);
            timer1.Interval = new TimeSpan(0, 0, 0, 0, 100);
        }

        private void timerTick(object sender, EventArgs e)
        {
            g.RemoveVisual(visual);
            using (dc = visual.RenderOpen())
            {
                // axis drawing
                axis.Draw(dc, visual);

                // Joints drawing
                Joint next = robot.joints[0];
                while (next != null)
                {
                    next.Update();
                    next.Draw(dc);
                    next = next.child;
                }

                // Target ball drawing
                dc.DrawEllipse(Brushes.Red, null, target, 3, 3);


                // Base drawing
                double rect = 5;
                dc.DrawRectangle(Brushes.Orange, null, new Rect(robot.joints[0].GetStartPos().X - rect / 2, robot.joints[0].GetStartPos().Y - rect, rect, rect * 2));

                dc.Close();
                g.AddVisual(visual);
            }
        }


        public void StartInverseKinematics()
        {
            if (timer1.IsEnabled)
            {
                timer1.Stop();
                movements = 0;
            }
            else
            {
                movements = 5000;
                timer1.Start();
            }
        }
        public void timer1_Tick(object sender, EventArgs e)
        {
            double[] angles = { robot.joints[0].angle, robot.joints[1].angle };

            Vector reachingPoint = new Vector(mouse.X, mouse.Y);
            angles = InverseKinematics(reachingPoint, angles);
            robot.joints[0].angle = angles[0];
            robot.joints[1].angle = angles[1];

            if ((--movements) <= 0)
            {
                timer1.Stop();
            }
        }
        public double[] InverseKinematics(Vector target, double[] angles)
        {
            if (DistanceFromTarget(target, angles) < DistanceThreshold)
            {
                movements = 0;
                return angles;
            }

            double[] oldAngles = { 0.0, 0.0 };
            angles.CopyTo(oldAngles, 0);
            for (int i = 0; i < 2; i++)
            {
                // Gradient descent
                // Update : Solution -= LearningRate * Gradient
                double gradient = PartialGradient(target, angles, i);
                angles[i] -= LearningRate * gradient;


                // Early termination
                var dist = DistanceFromTarget(target, angles);
                if (dist < DistanceThreshold || checkAngles(oldAngles, angles))
                {
                    movements = 0;
                    return angles;
                }
            }

            return angles;
        }
        public bool checkAngles(double[] oldAngles, double[] angles)
        {
            for (int i = 0; i < 2; i++)
            {
                if (oldAngles[i] != angles[i])
                    return false;
            }

            return true;
        }

        public double PartialGradient(Vector target, double[] angles, int i)
        {
            // Saves the angle,
            // it will be restored later
            double angle = angles[i];

            // Gradient : [F(x+SamplingDistance) - F(x)] / h
            double f_x = DistanceFromTarget(target, angles);

            robot.joints[i].angle += SamplingDistance;
            double f_x_plus_d = DistanceFromTarget(target, angles);

            double gradient = (f_x_plus_d - f_x) / SamplingDistance;

            // Restores
            robot.joints[i].angle = angle;

            return gradient;
        }

        public double DistanceFromTarget(Vector target, double[] angles)
        {
            Vector point = ForwardKinematics(angles);
            return Math.Sqrt(Math.Pow((point.X - target.X), 2.0) + Math.Pow((point.Y - target.Y), 2.0));
        }

        private void g_MouseLeftButtonUp(object sender, MouseButtonEventArgs e)
        {
            mouse = e.GetPosition(g);
            target = mouse;

            StartInverseKinematics();
        }

        public Vector ForwardKinematics(double[] angles)
        {
            Vector p = new Vector();

            var L1 = robot.joints[0].GetLen();
            var Q1 = robot.joints[0].angle + -Math.PI / 2;

            var L2 = robot.joints[1].GetLen();
            var Q2 = robot.joints[1].angle;

            // Formula:
            // x = XA + x' = L1*cos(Q1) + L2*cos(Q1+Q2)
            // y = YA + y' = L1*sin(Q1) + L2*sin(Q1+Q2)

            var x = robot.joints[0].GetStartPos().X + L1 * Math.Cos(Q1) + L2 * Math.Cos(Q1 + Q2);
            var y = robot.joints[0].GetStartPos().Y + L1 * Math.Sin(Q1) + L2 * Math.Sin(Q1 + Q2);

            p.X = Math.Round(x);
            p.Y = Math.Round(y);

            return p;
        }
    }

    class Robot
    {
        public List<Joint> joints = new List<Joint>();


        public Robot(double w, double h)
        {
            Joint joint_1 = new Joint(w / 2, h / 2, 50, 0);
            Joint joint_2 = new Joint(joint_1, 50, 0);
            joint_1.child = joint_2;

            joints.Add(joint_1);
            joints.Add(joint_2);
        }

        public double GetJointsLength() => joints.Sum(p => p.GetLen());

        public void Draw()
        {

        }
    }

    class Joint
    {
        Point pStart, pEnd;
        public double angle_, angle, len;
        Joint parent;
        Brush color = Brushes.White;
        public Joint child;
        public double angleMin = -180;
        public double angleMax = 180;

        public Joint(double x, double y, double length, double angle)
        {
            pStart = new Point(x, y);
            len = length;
            this.angle = Numerics.ToRadians(angle);
            parent = null;
            CalculateEndPoint();
        }

        public Joint(Joint parent, double length, double angle)
        {
            this.parent = parent;
            pStart = parent.pEnd;
            len = length;
            this.angle = Numerics.ToRadians(angle);
            CalculateEndPoint();
        }

        public Point GetStartPos() => pStart;
        public Point GetEndPos() => pEnd;
        public double GetLen() => len;
        public void Update()
        {
            angle_ = angle;
            if (parent != null)
            {
                pStart = parent.pEnd;
                angle_ += parent.angle_;
            }
            else
            {
                // to turn axis 90 degrees
                angle_ += -Math.PI / 2;
            }
            CalculateEndPoint();
        }
        void CalculateEndPoint()
        {
            double dx = len * Math.Cos(angle_);
            double dy = len * Math.Sin(angle_);
            pEnd = new Point(pStart.X + dx, pStart.Y + dy);
        }
        public void Draw(DrawingContext dc) => dc.DrawLine(new Pen(color, 2), pStart, pEnd);
    }

    static class Numerics
    {
        public static double ToRadians(double num) => num * Math.PI / 180.0;
        public static double ToDegrees(double num) => num * 180.0 / Math.PI;
        public static double GetDistance(Point a, Point b) => Math.Sqrt((a.X - b.X) * (a.X - b.X) + (a.Y - b.Y) * (a.Y - b.Y));
    }
}