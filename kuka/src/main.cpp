//standard c++ incldues
#include <iostream>
#include <string>
/*
 *  Theoretical diagram visualization
 *  ==================================
 *    Simulation -----------------> state ----------------------> Visualizer
 *         ^                          |
 *         |                          |
 *         |                          |
 *         |                          v
 *         ---------------------- Controller
 */

/*
 *  Drake diagram visualization
 *  ============================
 * 
 * 
 * 
 * 
 * 
 * 
 */

/*
 * Drake + Kuka IIWA Notes:
 * ----------------------
 *      - Kuka has 14 state variables with the iiwa14.sdf file we are using.
 *      - it is split by joint positions and velocities (7 joints + 7 joint velocities)
 *      - 7 linker robot.
 *
 *  Drake's Systems is designed to not be touched.
 *      - It describes f() in xdot = f() which is the ODE continuous equation or the discrete difference equation
 * 
 *  Drake's Context
 *      - Since we don't want to touch the system, we will use its context instead.
 *      - The context holds the runtime values
 *              - x[n+1] = f(n,x[n],u[n])
 *              - Context holds n,x,u
 *              - Sometimes can hold more inputs like noise.
 * 
 */

//for the drake "diagramming"
#include <drake/systems/framework/diagram_builder.h>
#include <drake/systems/framework/diagram.h>

//scene graph block
#include <drake/geometry/scene_graph.h> 

//multibody is robot block
#include <drake/multibody/tree/multibody_element.h>

//parser to get multibody from a urdf/sdf file using filepath std::string 
#include <drake/multibody/parsing/parser.h>

//drake default visualizer
#include <drake/geometry/drake_visualizer.h>

//simulator
#include <drake/systems/analysis/simulator.h>

namespace drake {

int setupDrake()
{
    // object that creates all of the "diagram blocks"
    systems::DiagramBuilder<double> builder;
    
    //add multibody plant block (empty)
    //time_step = 1e-4 second (discrete setting)
    //time_step = 0 second (continuous setting)
    // provides with the difference equation or continuous ode
    //multibody::MultibodyPlant<double> robot(1e-4); 
    multibody::MultibodyPlant<double>* robot{};

    //add scenegraph block (for geometry)
    geometry::SceneGraph<double>* scene_graph;
    std::tie(robot, scene_graph) = multibody::AddMultibodyPlantSceneGraph(&builder, 1e-4);

    // requires -> multibodyplant block and (optionally) scene graph block
    multibody::Parser parser(robot);
    parser.AddModelFromFile("/opt/drake/share/drake/manipulation/models/iiwa_description/sdf/iiwa14_no_collision.sdf");

    //weld iiwa bot to the ground
    robot->WeldFrames(robot->world_frame(), robot->GetFrameByName("iiwa_link_0"));
    robot->Finalize();
    //std::unique_ptr<systems::Context<double>> robot_context = robot->CreateDefaultContext();


    geometry::DrakeVisualizer<double>::AddToBuilder(&builder,*scene_graph);

    auto diagram = builder.Build();
    std::unique_ptr<systems::Context<double>> context = diagram->CreateDefaultContext();
    systems::Context<double>& robot_context = robot->GetMyMutableContextFromRoot(context.get());

    Eigen::VectorXd joint_pos = Eigen::VectorXd::Zero(7);
    joint_pos(0) = -1.57;
    joint_pos(1) = 0.1;
    joint_pos(3) = -1.2;
    joint_pos(5) = 1.6;
    robot->SetPositions(&robot_context, joint_pos);

    joint_pos = Eigen::VectorXd::Zero(7);

    robot->get_actuation_input_port().FixValue(&robot_context, joint_pos);

    // simulator runs RK-4 or whatever integration method
    systems::Simulator<double> simulator(*diagram, std::move(context));
    simulator.set_target_realtime_rate(1.0);
    simulator.AdvanceTo(10.0);
    
    return 0;
}

}//namespace drake


// if using drake visualization, run /opt/drake/bin/drake_visualizer to see output.
// don't use meshcat for now ;{
int main()
{
    std::cout << "Hello World!\n";
    drake::setupDrake();
    return 0;
}