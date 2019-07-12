using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using RobotOrientAddIn;
using RobotOM;

namespace DebugRobotAddIn
{
    class Program
    {
        static void Main()
        {
            string filename = "V:\\1979\\Analysis\\Robot\\Curved glass\\190617 wireframe model\\wireframe_model vQ.rtd";

            var add_in = new RobotOrientAddIn.RobotOrientAddIn();
            var robot_app = new RobotApplication
            {
                Visible = 1,
                Interactive = 1
            };
            robot_app.Project.Open(filename);
            int add_in_id = 0;
            var is_connected = add_in.Connect(robot_app, add_in_id, true);
            if (is_connected)
                add_in.DoCommand(0);
            else
                Console.Write("Couldn't connect");

            Console.Write("Press return key to continue...");
            Console.ReadLine();
        }
    }
}
