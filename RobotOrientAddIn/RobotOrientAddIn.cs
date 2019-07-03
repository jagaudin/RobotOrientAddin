using System;
using System.Collections.Generic;
using MathNet.Spatial.Euclidean;
using MathNet.Spatial.Units;
using RobotOM;


namespace RobotOrientAddIn
{
    [System.Runtime.InteropServices.ComVisibleAttribute(true), System.Runtime.InteropServices.Guid("0588E2AC-5F9E-49EF-9BD9-671B01CD8BE1")]

    public class DefaultDict<TKey, TValue>: Dictionary<TKey, TValue> where TValue: new()
    {
        public new TValue this[TKey key]
        {
            get
            {
                if (!TryGetValue(key, out TValue val))
                {
                    val = new TValue();
                    Add(key, val);
                }
                return val;
            }
            set { base[key] = value; }
        }
    }
    

    public class RobotOrientAddIn: IRobotAddIn
    {

        private IRobotApplication robot_app = null;
        public double GetExpectedVersion()
        {
            return 0;
        }

        public bool Connect(RobotApplication robot_app, int add_in_id, bool first_time)
        {
            if (robot_app == null) return false;
            this.robot_app = robot_app;
            return true;
        }
        public bool Disconnect()
        {
            robot_app = null;
            return true;
        }

        public int InstallCommands(RobotCmdList cmd_list)
        {
            cmd_list.New(1, "Orient grid elements");
            return cmd_list.Count;
        }

        public void DoCommand(int cmd_id)
        {
            IRobotStructure structure = robot_app.Project.Structure;

            // Get bars and nodes
            IRobotCollection bars = structure.Bars.GetAll();
            IRobotCollection nodes = structure.Nodes.GetAll();

            // Create 3D points at nodes
            var points = new Dictionary<int, Point3D>();
            for (int i = 1; i <= nodes.Count; i++)
            {
                var node = (IRobotNode)nodes.Get(i);
                points[i] = new Point3D(node.X, node.Y, node.Z);
            }

            // Create 3D vectors for each bar and index of bars connected to a node
            var vectors = new Dictionary<int, Vector3D>();
            var vect_by_pt = new DefaultDict<int, List<Vector3D>>();
            for (int i = 1; i <= bars.Count; i++)
            {
                var bar = (IRobotBar)bars.Get(i);
                var start_pt = points[bar.StartNode];
                var end_pt = points[bar.EndNode];
                vectors[i] = end_pt - start_pt;
                vect_by_pt[bar.StartNode].Add(vectors[i]);
                vect_by_pt[bar.EndNode].Add(vectors[i]);
            };

            foreach (KeyValuePair<int, Vector3D> vector in vectors)
            {
                // `u` is the vector corresponding to the bar
                Vector3D u = vector.Value;
                UnitVector3D u_norm = u.Normalize();
                int start = bars.Get(vector.Key).StartNode;

                // Find the most orthogonal vector `v`
                Vector3D most_orth_v = u;
                double cur_min = 1;
                foreach (Vector3D x in vect_by_pt[start])
                {
                    UnitVector3D x_norm = x.Normalize();
                    double dot_prod = Math.Abs(u_norm.DotProduct(x_norm));
                    if (dot_prod < cur_min)
                    {
                        most_orth_v = x;
                        cur_min = dot_prod;
                    }
                }

                if (cur_min > 0.95) continue;

                var v = most_orth_v;
                var v_norm = v.Normalize();

                // Vector `a` is vector a orthogonal to `u` in (u,v) plane
                Vector3D a = v - u_norm.DotProduct(v) * u;
                UnitVector3D a_norm = a.Normalize();

                // Vector `c` is orthogonal to `u` in the global (X,Y) plane
                UnitVector3D c = u_norm.CrossProduct(UnitVector3D.ZAxis);
                // Vector `d` is orthogonal to `c` and `u`
                UnitVector3D d = c.CrossProduct(u_norm);

                // Calculate the angles of `a` with `d` and `c`
                Angle theta1 = a.AngleTo(d);
                Angle theta2 = a.AngleTo(c);

                // Calculate gamma from `theta1` and `theta2`
                Angle gamma = (theta2.Degrees < 90) ? theta1 : -theta1;
                double gamma_up = (gamma.Degrees < 0) ? gamma.Degrees + 90 : gamma.Degrees - 90;

                // Set `Gamma` attribute of bar
                IRobotBar bar = bars.Get(vector.Key);
                bar.Gamma = gamma_up;
            }

            // Redraw all views
            robot_app.Project.ViewMngr.Refresh();
        }
    }
}
