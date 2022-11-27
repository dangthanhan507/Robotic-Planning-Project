#include <iostream>

#include <gflags/gflags.h>

#include <drake/systems/framework/event.h>

//common submodule
#include "drake/common/drake_assert.h"
#include "drake/common/find_resource.h"

//visual + scene_graph
#include "drake/geometry/scene_graph.h"
#include "drake/geometry/drake_visualizer.h"

//lcm
#include "drake/lcm/drake_lcm.h"

//multibody
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/plant/multibody_plant.h"

//misc
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/controllers/linear_quadratic_regulator.h"

//diagram for systems
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/framework/framework_common.h"

//basic systems
#include <drake/systems/framework/basic_vector.h>
#include "drake/systems/primitives/affine_system.h"
#include "drake/systems/primitives/constant_vector_source.h"
#include "drake/systems/primitives/linear_system.h"


namespace drake {
namespace examples {
namespace multibody {
namespace cart_pole {
namespace {

DEFINE_double(target_realtime_rate, 1.0,
              "Rate at which to run the simulation, relative to realtime");
DEFINE_double(simulation_time, 10, "How long to simulate the pendulum");
DEFINE_double(max_time_step, 1.0e-3,
              "Simulation time step used for integrator.");

//cart pole sdf
static const char* const kCartPoleSdfPath = "cartpole.sdf";

int DoMain()
{
    DRAKE_DEMAND(FLAGS_simulation_time > 0);

    //creates diagram essential for drake
    systems::DiagramBuilder<double> builder;
    
    geometry::SceneGraph<double>& scene_graph = *builder.AddSystem<geometry::SceneGraph>();
    scene_graph.set_name("scene_graph");        

    drake::multibody::MultibodyPlant<double>* cp = builder.AddSystem<drake::multibody::MultibodyPlant<double>>(FLAGS_max_time_step);
    cp->set_name("cart_pole");
    cp->RegisterAsSourceForSceneGraph(&scene_graph);

    drake::multibody::Parser parser(cp);
    //const std::string sdf_path = FindResource(kCartPoleSdfPath);
    drake::multibody::ModelInstanceIndex plant_model_instance_index = parser.AddModelFromFile(kCartPoleSdfPath);
    (void)plant_model_instance_index;

    //finish the plant
    cp->Finalize();

    auto cp_context = cp->CreateDefaultContext();
    const systems::InputPort<double>& actuation_port = cp->get_actuation_input_port();

    Eigen::VectorXd u0_e = Eigen::VectorXd::Zero(2);
    systems::BasicVector<double> u0(u0_e);
    actuation_port.FixValue(cp_context.get(), u0);

    //Eigen::VectorXd u0 = Eigen::VectorXd::Zero(2);
    //cp_context->FixInputPort(cartpole_actuation_port, u0);

    Eigen::VectorXd x0 = Eigen::VectorXd::Zero(4);
    x0[0] = 1;
    x0[1] = M_PI;
    cp_context->SetDiscreteState(x0);

    Eigen::MatrixXd Q = Eigen::MatrixXd::Identity(4,4);
    Q(0,0) = 10;
    Q(1,1) = 10;
    Eigen::MatrixXd R = Eigen::MatrixXd::Identity(2,2);
    Eigen::MatrixXd N;
    auto lqr = builder.AddSystem(systems::controllers::LinearQuadraticRegulator(*cp,*cp_context,Q,R,N,actuation_port.get_index()));

    builder.Connect(cp->get_state_output_port(), lqr->get_input_port());
    builder.Connect(lqr->get_output_port(), cp->get_actuation_input_port());

    DRAKE_DEMAND(!!cp->get_source_id());
    builder.Connect(cp->get_geometry_poses_output_port(), scene_graph.get_source_pose_port(cp->get_source_id().value()));
    builder.Connect(scene_graph.get_query_output_port(),cp->get_geometry_query_input_port());

    geometry::DrakeVisualizer<double>::AddToBuilder(&builder,scene_graph);

    auto diagram = builder.Build();
    std::unique_ptr<systems::Context<double>> diagram_context = diagram->CreateDefaultContext();

    systems::Context<double>& plant_context = diagram->GetMutableSubsystemContext(*cp, diagram_context.get());

    Eigen::VectorXd positions = Eigen::VectorXd::Zero(2);
    positions[0] = 0.0;
    positions[1] = 1.0;
    cp->SetPositions(&plant_context, positions);

    systems::Simulator<double> simulator(*diagram, std::move(diagram_context));
    simulator.set_publish_every_time_step(true);
    simulator.set_target_realtime_rate(FLAGS_target_realtime_rate);
    simulator.Initialize();
    simulator.AdvanceTo(FLAGS_simulation_time);

    return 0;
}



} //namespace drake
} //namespace examples
} //namespace multibody
} //namespace cart_pole
} //namespace

int main(int argc, char* argv[])
{
    //gflags::ParseCommandLineFlags(&argc, &argv, true);
    drake::examples::multibody::cart_pole::DoMain();
    return 0;
}