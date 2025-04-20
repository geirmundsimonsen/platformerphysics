use rapier2d_f64::prelude::*;
use std::ffi::c_void;
use crate::ppm;

pub struct TestEventHandler {

}

impl TestEventHandler {
  pub fn new() -> Self {
    TestEventHandler {

    }
  }
}

impl EventHandler for TestEventHandler {
  fn handle_collision_event(&self, bodies: &RigidBodySet, colliders: &ColliderSet, event: CollisionEvent, contact_pair: Option<&ContactPair>) {
    println!("Collision event: {:?}", event);
    println!("{:?}", contact_pair.unwrap().max_impulse());
    
  }

  fn handle_contact_force_event(&self, dt: f64, bodies: &RigidBodySet, colliders: &ColliderSet, contact_pair: &ContactPair, total_force_magnitude: f64) {
    println!("{:?}", contact_pair.max_impulse());
    println!("Contact force event: {} {} {} {}", contact_pair.collider1.into_raw_parts().0, contact_pair.collider2.into_raw_parts().0, dt, total_force_magnitude);
  }
}

pub struct EventTestContext {
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
  pub event_handler: TestEventHandler,
  pub counter: usize,
}

impl EventTestContext {
  pub fn new() -> Self {
    let mut ip = IntegrationParameters::default();
    ip.dt = 1.0 / 240.0; // 240 hz

    let mut rigid_body_set = RigidBodySet::new();
    let mut collider_set = ColliderSet::new();
    let mut impulse_joint_set = ImpulseJointSet::new();

    let rb_ball = rigid_body_set.insert(RigidBodyBuilder::dynamic().translation(vector![0.0, 2.5]).build());
    collider_set.insert_with_parent(ColliderBuilder::ball(0.5).restitution(0.4).active_events(ActiveEvents::all()).build(), rb_ball, &mut rigid_body_set);

    collider_set.insert(ColliderBuilder::cuboid(3.0, 0.1).build()); 

    EventTestContext {
      rigid_body_set,
      collider_set,
      gravity: vector![0.0, -9.81],
      integration_parameters: ip,
      physics_pipeline: PhysicsPipeline::new(),
      island_manager: IslandManager::new(),
      broad_phase: DefaultBroadPhase::new(),
      narrow_phase: NarrowPhase::new(),
      impulse_joint_set,
      multibody_joint_set: MultibodyJointSet::new(),
      ccd_solver: CCDSolver::new(),
      query_pipeline: QueryPipeline::new(),
      // Add physics_hooks and event_handler if they have state or need specific setup
      // physics_hooks: YourPhysicsHooks,
      event_handler: TestEventHandler::new(),
      counter: 0,
    }
  }

  pub fn advance(&mut self) {
    let physics_hooks = ();

    println!("counter: {}", self.counter);

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
      &self.event_handler
    );

    self.rigid_body_set.iter().for_each(
      |(handle, rb)| {
        println!("Rigid body {} position: {:?}", handle.into_raw_parts().0, rb.position().translation);
      }
    );

    self.counter += 1;
  }
}

#[cfg(test)]
mod tests {
  use super::*;

  use crate::ppm;

  #[test]
  fn test() {
    let foo = ppm::PPM::new(100, 100);
    foo.write_file("test.ppm").unwrap();

    let mut swc = EventTestContext::new();
    for i in 0..240 {
      swc.advance();
    }
  }
}