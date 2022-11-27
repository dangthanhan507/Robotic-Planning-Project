#include <iostream>
#include <gflags/gflags.h>
//common submodule
#include "drake/common/drake_assert.h"
#include "drake/common/is_approx_equal_abstol.h"

//using example
#include "drake/examples/pendulum/pendulum_geometry.h"
#include "drake/examples/pendulum/pendulum_plant.h"

//visualizer submodule
#include "drake/geometry/drake_visualizer.h"

//systems submodules
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/controllers/linear_quadratic_regulator.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/framework/input_port.h"

namespace drake {
namespace examples {
namespace pendulum {

DEFINE_double(target_realtime_rate, 1.0,  "Playback speed.  See documentation for Simulator::set_target_realtime_rate() for details.");

int DoMain()
{
    //creates diagram essential for drake
    systems::DiagramBuilder<double> builder;

    PendulumPlant<double>* pendulum = builder.AddSystem<PendulumPlant>();
    pendulum->set_name("pendulum");

    std::unique_ptr<systems::Context<double>> pendulum_context = pendulum->CreateDefaultContext();
    PendulumState<double>& desired_state = pendulum->get_mutable_state(pendulum_context.get());

    //inversion of pendulum + stabilization
    desired_state.set_theta(M_PI);
    desired_state.set_thetadot(0);

    pendulum->get_input_port().FixValue(pendulum_context.get(), PendulumInput<double>{}.with_tau(0.0));

    // setup lqr cost function: integral of 10*theta^2 + thetadot^2 + tau^2
    Eigen::MatrixXd Q(2,2); // state cost matrix
    Q << 10, 0 ,0 ,1; //input row order
    Eigen::MatrixXd R(1,1); // control input cost matrix
    R << 1;
    
    //lol just throw in the LQR
    systems::AffineSystem<double>* controller = builder.AddSystem(systems::controllers::LinearQuadraticRegulator(*pendulum, *pendulum_context, Q, R));
    controller->set_name("controller");
    builder.Connect(pendulum->get_state_output_port(), controller->get_input_port()); //basically pipe state into lqr
    builder.Connect(controller->get_output_port(), pendulum->get_input_port()); // pipe control input into pendulum sim

    geometry::SceneGraph<double>* scene_graph = builder.AddSystem<geometry::SceneGraph>(); //create a diagram block
    geometry::DrakeVisualizer<double>::AddToBuilder(&builder,*scene_graph);



    //build up the diagram after all connections are made
    std::unique_ptr<drake::systems::Diagram<double>> diagram = builder.Build();

    systems::Simulator<double> simulator(*diagram);
    systems::Context<double>& sim_pendulum_context = diagram->GetMutableSubsystemContext(*pendulum, &simulator.get_mutable_context());
    PendulumState<double>& state = pendulum->get_mutable_state(&sim_pendulum_context);
    state.set_theta(M_PI+0.1); //set initial sstate of pendulum
    state.set_thetadot(0.2); 

    simulator.set_target_realtime_rate(FLAGS_target_realtime_rate);
    

    simulator.Initialize();

    //mesh cat start recording while simulator is moving
    simulator.AdvanceTo(10);

    DRAKE_DEMAND( is_approx_equal_abstol(state.value(), desired_state.value(), 1e-3) );

    return 0;
}



} //namespace drake
} //namespace examples
} //namespace pendulum

int main(int argc, char* argv[])
{
    //gflags::ParseCommandLineFlags(&argc, &argv, true);
    drake::examples::pendulum::DoMain();
    return 0;
}