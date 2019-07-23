using System;
using System.Collections.Generic;
using MathNet.Spatial.Euclidean;
using MathNet.Spatial.Units;
using RobotOM;


namespace RobotOrientAddIn
{
    [System.Runtime.InteropServices.ComVisibleAttribute(false), System.Runtime.InteropServices.Guid("0588E2AC-5F9E-49EF-9BD9-671B01CD8BE1")]
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

    [System.Runtime.InteropServices.ComVisibleAttribute(true), System.Runtime.InteropServices.Guid("2547EE18-E1EA-4AC9-88A3-E935D95BFF41")]
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
            OrientBars();
        }

        public void OrientBars(IRobotSelection bar_selection=null)
        {
            IRobotStructure structure = robot_app.Project.Structure;
            // If no parameter, get the current bar selection, if none selected, work on all bars
            if (bar_selection != null)
            {
                bar_selection = structure.Selections.Get(IRobotObjectType.I_OT_BAR);
                if (bar_selection.Count == 0)
                    bar_selection = structure.Selections.CreateFull(IRobotObjectType.I_OT_BAR);
            }
            
            // Get nodes as IRobotCollection
            //IRobotCollection nodes = structure.Nodes.GetAll();
            IRobotCollection bars = structure.Bars.GetAll();


            // Create 3D points at nodes
            var points = new Dictionary<int, Point3D>();
            // Create 3D vectors for each bar and index of bars connected to a node
            var vectors = new Dictionary<int, Vector3D>();
            var vect_by_pt = new DefaultDict<int, List<Vector3D>>();

            for (int i = 1; i <= bars.Count; i++)
            {
                var bar = (IRobotBar)bars.Get(i);
                var bar_number = bar.Number;
                var end_nodes = new List<IRobotNode> {
                    (IRobotNode)structure.Nodes.Get(bar.StartNode), 
                    (IRobotNode)structure.Nodes.Get(bar.EndNode)
                };

                foreach (IRobotNode end_node in end_nodes)
                {
                    var end_node_number = end_node.Number;
                    if (!points.ContainsKey(end_node_number))
                    {
                        points[end_node_number] = new Point3D(end_node.X, end_node.Y, end_node.Z);
                    }
                }
                vectors[bar_number] = points[end_nodes[1].Number] - points[end_nodes[0].Number];

                foreach (IRobotNode end_node in end_nodes)
                {
                    vect_by_pt[end_node.Number].Add(vectors[bar_number]);
                }
            }

            // Iterate over selected bars
            for (int i = 1; i <= bar_selection.Count; i++)
            {
                var bar_number = bar_selection.Get(i);
                // `u_norm` is the unit vector corresponding to the bar
                UnitVector3D u_norm = vectors[bar_number].Normalize();

                if (u_norm.IsParallelTo(UnitVector3D.ZAxis))
                    continue;

                var bar = (IRobotBar)structure.Bars.Get(bar_number);
                int start = bar.StartNode;
                // TODO: How about the other end?

                // Find the most orthogonal vector `v`
                UnitVector3D most_orth_v = u_norm;
                double cur_min = 1;

                foreach (Vector3D x in vect_by_pt[start])
                {
                    UnitVector3D x_norm = x.Normalize();
                    double dot_prod = Math.Abs(u_norm.DotProduct(x_norm));
                    if (dot_prod < cur_min)
                    {
                        most_orth_v = x_norm;
                        cur_min = dot_prod;
                    }
                }

                if (cur_min > 0.95) continue;

                var v_norm = most_orth_v;

                // Vector `a` is vector a orthogonal to `u` in (u,v) plane
                Vector3D a = v_norm - u_norm.DotProduct(v_norm) * u_norm;

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
                bar.Gamma = gamma_up;
            }

            // Redraw all views
            robot_app.Project.ViewMngr.Refresh();
        }
    }
}
