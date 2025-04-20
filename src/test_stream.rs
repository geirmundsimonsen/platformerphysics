use rapier2d_f64::prelude::*;

pub struct PhysicsContext {
  pub rigid_body_set: RigidBodySet,
  pub collider_set: ColliderSet,
  pub gravity: Vector<f64>,
  pub integration_parameters: IntegrationParameters,
  pub physics_pipeline: PhysicsPipeline,
  pub island_manager: IslandManager,
  pub broad_phase: DefaultBroadPhase,
  pub narrow_phase: NarrowPhase,
  pub impulse_joint_set: ImpulseJointSet,
  pub multibody_joint_set: MultibodyJointSet,
  pub ccd_solver: CCDSolver,
  pub query_pipeline: QueryPipeline,
}

impl PhysicsContext {
  pub fn new() -> Self {
    PhysicsContext {
      rigid_body_set: RigidBodySet::new(),
      collider_set: ColliderSet::new(),
      gravity: vector![0.0, -9.81],
      integration_parameters: IntegrationParameters::default(),
      physics_pipeline: PhysicsPipeline::new(),
      island_manager: IslandManager::new(),
      broad_phase: DefaultBroadPhase::new(),
      narrow_phase: NarrowPhase::new(),
      impulse_joint_set: ImpulseJointSet::new(),
      multibody_joint_set: MultibodyJointSet::new(),
      ccd_solver: CCDSolver::new(),
      query_pipeline: QueryPipeline::new(),
      // Add physics_hooks and event_handler if they have state or need specific setup
      // physics_hooks: YourPhysicsHooks,
      // event_handler: YourEventHandler,
    }
  }

  pub fn step(&mut self) {
    // Note: Using () for hooks/handlers assumes they are stateless ZSTs.
    // If they have state or logic, you'll need to handle them appropriately.
    let physics_hooks = ();
    let event_handler = ();

    self.physics_pipeline.step(
      &self.gravity,
      &self.integration_parameters,
      &mut self.island_manager,
      &mut self.broad_phase,
      &mut self.narrow_phase,
      &mut self.rigid_body_set,
      &mut self.collider_set,
      &mut self.impulse_joint_set,
      &mut self.multibody_joint_set,
      &mut self.ccd_solver,
      Some(&mut self.query_pipeline),
      &physics_hooks, // Pass your actual hooks if needed
      &event_handler, // Pass your actual event handler if needed
    );
  }

  // Example method to get ball altitude (you'll need to store the handle)
  // You might want to pass handles/IDs from C++ or store them in the context
  pub fn get_body_translation_y(&self, handle_index: u32, handle_generation: u32) -> Option<f64> {
    let handle = RigidBodyHandle::from_raw_parts(handle_index, handle_generation);
    self.rigid_body_set.get(handle).map(|body| body.translation().y)
  }

  // Add other methods as needed: add_body, remove_body, get_collider_position, etc.
  pub fn add_ball_and_ground(&mut self) -> RigidBodyHandle {
    // Create the ground.
    let collider = ColliderBuilder::cuboid(100.0, 0.1).build();
    self.collider_set.insert(collider);

    // Create the bouncing ball.
    let rigid_body = RigidBodyBuilder::dynamic()
      .translation(vector![0.0, 10.0])
      .build();
    let collider = ColliderBuilder::ball(0.5).restitution(0.7).build();
    let ball_body_handle = self.rigid_body_set.insert(rigid_body);
    self.collider_set.insert_with_parent(collider, ball_body_handle, &mut self.rigid_body_set);

    ball_body_handle
  }

  pub fn add_rigid_body_dynamic(&mut self, pos_x: f64, pos_y: f64) -> RigidBodyHandle {
    let rigid_body = RigidBodyBuilder::dynamic()
      .translation(vector![pos_x, pos_y])
      .build();
    self.rigid_body_set.insert(rigid_body)
  }

  pub fn add_collider_cuboid(&mut self, half_extents_x: f64, half_extents_y: f64) -> ColliderHandle {
    let collider = ColliderBuilder::cuboid(half_extents_x, half_extents_y).build();
    self.collider_set.insert(collider)
  }

  pub fn add_collider_ball(&mut self, radius: f64, restitution: f64) -> ColliderHandle {
    let collider = ColliderBuilder::ball(radius).restitution(restitution).build();
    self.collider_set.insert(collider)
  }

  pub fn add_collider_ball_with_parent(&mut self, rigid_body_handle: RigidBodyHandle, radius: f64, restitution: f64) -> ColliderHandle {
    let collider = ColliderBuilder::ball(radius).restitution(restitution).build();
    self.collider_set.insert_with_parent(collider, rigid_body_handle, &mut self.rigid_body_set)
  }
}

#[cfg(test)]
mod tests {
  use super::*;

  #[ignore]
  #[test]
  fn test() {
    let mut pc = PhysicsContext::new();

    pc.add_collider_cuboid(100.0, 0.1);
    let handle = pc.add_rigid_body_dynamic(0.0, 9998.0);
    pc.add_collider_ball_with_parent(handle, 0.5, 0.7);
  
    for _ in 0..3000 {
      pc.step();
      let (index, generation) = handle.into_raw_parts();
      println!("Ball altitude: {}", pc.get_body_translation_y(index, generation).unwrap());
    }
  }
}