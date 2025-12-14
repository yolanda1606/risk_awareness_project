import numpy as np

# Import the necessary Drake components
from pydrake.geometry import StartMeshcat, Box
from pydrake.multibody.plant import AddMultibodyPlantSceneGraph, CoulombFriction
from pydrake.multibody.tree import SpatialInertia, UnitInertia
from pydrake.systems.analysis import Simulator
from pydrake.systems.framework import DiagramBuilder
from pydrake.visualization import AddDefaultVisualization

# <--- CHANGED: Added RollPitchYaw to imports
from pydrake.math import RigidTransform, RollPitchYaw 

def main():
    # 1. Start the visualizer (Meshcat)
    meshcat = StartMeshcat()
    
    # 2. Create the DiagramBuilder
    builder = DiagramBuilder()
    
    # 3. Add the Physics Plant
    # We use a time step of 1ms
    plant, scene_graph = AddMultibodyPlantSceneGraph(builder, time_step=0.001)

    # --- A. ADD THE GROUND ---
    plant.RegisterVisualGeometry(
        plant.world_body(),
        RigidTransform(),
        Box(10, 10, 0.1),
        "ground_visual",
        [0.5, 0.5, 0.5, 1.0]) 

    plant.RegisterCollisionGeometry(
        plant.world_body(),
        RigidTransform(),
        Box(10, 10, 0.1),
        "ground_collision",
        CoulombFriction(1.0, 1.0))

    # --- B. ADD THE FALLING BOX ---
    # 1. Define Mass/Inertia (1.0 kg mass, simple cube inertia)
    box_inertia = SpatialInertia(mass=1.0, 
                                 p_PScm_E=[0., 0., 0.], 
                                 G_SP_E=UnitInertia.SolidBox(0.2, 0.2, 0.2))
    
    # 2. Add the Rigid Body to the plant
    box_body = plant.AddRigidBody("my_falling_box", box_inertia)
    
    # 3. Add Shape (Visual)
    plant.RegisterVisualGeometry(
        box_body,
        RigidTransform(),
        Box(0.2, 0.2, 0.2), # 20cm cube
        "box_visual",
        [0.8, 0.1, 0.1, 1.0]) # Red color
        
    # 4. Add Collision (Physics)
    plant.RegisterCollisionGeometry(
        box_body,
        RigidTransform(),
        Box(0.2, 0.2, 0.2),
        "box_collision",
        CoulombFriction(0.5, 0.5))

    # 5. Finalize the plant
    plant.Finalize()
    
    # 6. Add Visualization
    AddDefaultVisualization(builder, meshcat)
    
    # 7. Build the Diagram
    diagram = builder.Build()
    
    # 8. Set up the Simulator
    simulator = Simulator(diagram)
    context = simulator.get_mutable_context()
    
    # Set the initial position of the box
    plant_context = plant.GetMyContextFromRoot(context)
    
    # <--- CHANGED: Wrapped rotation in RollPitchYaw(...)
    # Position: x=0, y=0, z=1.0 meter high
    # Rotation: Tilted 0.8 radians on X axis
    initial_pose = RigidTransform(RollPitchYaw(0.8, 0, 0), [0, 0, 1.0])
    
    plant.SetFreeBodyPose(plant_context, box_body, initial_pose)

    # 9. Run the simulation
    simulator.set_target_realtime_rate(1.0)
    
    print(f"Simulation started! Open this URL in your browser: {meshcat.web_url()}")
    
    # Run for 5 seconds
    simulator.AdvanceTo(5.0)
    print("Simulation finished.")

if __name__ == "__main__":
    main()