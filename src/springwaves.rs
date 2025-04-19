use rapier2d_f64::prelude::*;
use std::ffi::c_void;

pub struct SpringwaveContext {
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
  pub gametime: f64,
  pub phase: f64,
  pub rb_sine: RigidBodyHandle,
  pub rb_link1: RigidBodyHandle,
  pub rb_link2: RigidBodyHandle,
}

impl SpringwaveContext {
  pub fn new(gametime: f64) -> Self {
    let mut ip = IntegrationParameters::default();
    ip.dt = 1.0 / 240.0; // 240 hz

    let mut rigid_body_set = RigidBodySet::new();
    let mut collider_set = ColliderSet::new();
    let mut impulse_joint_set = ImpulseJointSet::new();
    let rb_sine = rigid_body_set.insert(RigidBodyBuilder::kinematic_position_based().translation(vector![0.0, 0.5]).build());
    //let collider = ColliderBuilder::ball(0.2).build();
    //collider_set.insert_with_parent(collider, rb_sine, &mut rigid_body_set);

    //let collider = ColliderBuilder::cuboid(10.0, 0.1).translation(vector![0.0, 0.0]).build();
    //collider_set.insert(collider);


    let rb_link1 = rigid_body_set.insert(RigidBodyBuilder::dynamic().translation(vector![1.0, -0.5]).additional_mass(1.0).build());
  
    /*impulse_joint_set.insert(rb_sine, rb_link1, PrismaticJointBuilder::new(-Vector::y_axis())
    .local_anchor1(point![0.0, 0.0])
    .local_anchor2(point![0.0, 0.0])
    .motor_position(-0.0, 110.79, 0.0)
    , true);*/

    /*impulse_joint_set.insert(rb_sine, rb_link1, RevoluteJointBuilder::new()
      //.limits([-360.0, 360.0])
      //.motor_position(4.0, 4.0, 0.2)
      .local_anchor1(point![0.0, -0.10])
      .local_anchor2(point![0.0, 0.9])
      .build()
    , true);*/

    impulse_joint_set.insert(rb_sine, rb_link1, RopeJointBuilder::new(1.0).build(), true);

    let rb_link2 = rigid_body_set.insert(RigidBodyBuilder::dynamic().translation(vector![0.0, -1.5]).additional_mass(1.0).build());

    /*impulse_joint_set.insert(rb_link1, rb_link2, PrismaticJointBuilder::new(-Vector::y_axis())
    .local_anchor1(point![0.0, 0.0])
    .local_anchor2(point![0.0, 0.0])
    .motor_position(-0.0, 2.3, 0.0)
    , true);
*/

    /*impulse_joint_set.insert(rb_link1, rb_link2, RevoluteJointBuilder::new()
    //.limits([-360.0, 360.0])
    //.motor_position(4.0, 4.0, 0.2)
    .local_anchor1(point![0.0, -0.10])
    .local_anchor2(point![0.0, 0.9])
    .build()
    , true);*/

    impulse_joint_set.insert(rb_link1, rb_link2, RopeJointBuilder::new(1.0).build(), true);

    SpringwaveContext {
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
      gametime,
      phase: 0.0,
      rb_sine,
      rb_link1,
      rb_link2,
      // Add physics_hooks and event_handler if they have state or need specific setup
      // physics_hooks: YourPhysicsHooks,
      // event_handler: YourEventHandler,
    }
  }

  pub fn advance(&mut self, gametime: f64) {
    while self.gametime < gametime {
      self.gametime += self.integration_parameters.dt;
      self.phase += self.integration_parameters.dt * 0.5;
      if self.phase > 1.0 {
        self.phase -= 1.0;
      }

      self.rigid_body_set[self.rb_sine].set_next_kinematic_translation(vector![0.0, (self.phase * 2.0 * std::f64::consts::PI).sin()]);

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

      //println!("rb_sine: {:>+0width$.prec$}", self.rigid_body_set[self.rb_sine].position().translation.y, width=8, prec=3);
      //println!("rb_link1: {:>+0width$.prec$}", self.rigid_body_set[self.rb_link1].position().translation.y, width=8, prec=3);
      /*println!("rb_sine: {:>+0width$.prec$}, rb_link1: {:>+0width$.prec$}, diff: {:>+0width$.prec$}",
        self.rigid_body_set[self.rb_sine].position().translation.y,
        self.rigid_body_set[self.rb_link1].position().translation.y,
        self.rigid_body_set[self.rb_sine].position().translation.y - self.rigid_body_set[self.rb_link1].position().translation.y,
        width=8, prec=3
      );*/
      println!("rb_sine - x: {:>+0width$.prec$}, y: {:>+0width$.prec$}, rot: {:>+0width$.prec$}, rb_link1 - x: {:>+0width$.prec$}, y: {:>+0width$.prec$}, rot: {:>+0width$.prec$}, rb_link2 - x: {:>+0width$.prec$}, y: {:>+0width$.prec$}, rot: {:>+0width$.prec$}",
        self.rigid_body_set[self.rb_sine].position().translation.x,
        self.rigid_body_set[self.rb_sine].position().translation.y,
        self.rigid_body_set[self.rb_sine].rotation().re,
        self.rigid_body_set[self.rb_link1].position().translation.x,
        self.rigid_body_set[self.rb_link1].position().translation.y,
        self.rigid_body_set[self.rb_link1].rotation().re,
        self.rigid_body_set[self.rb_link2].position().translation.x,
        self.rigid_body_set[self.rb_link2].position().translation.y,
        self.rigid_body_set[self.rb_link2].rotation().re,
        width=8, prec=3
      );
    }


  }
}

#[no_mangle]
pub extern "C" fn create_springwave(gametime: f64) -> *mut c_void {
  let context = Box::new(SpringwaveContext::new(gametime));
  Box::into_raw(context) as *mut c_void
}

#[no_mangle]
pub extern "C" fn advance_springwave(context_ptr: *mut c_void, gametime: f64, out_rb_sine_x: *mut f64, out_rb_sine_y: *mut f64, out_rb_link1_x: *mut f64, out_rb_link1_y: *mut f64, out_rb_link2_x: *mut f64, out_rb_link2_y: *mut f64) {
  if context_ptr.is_null() {
    eprintln!("Error: step_physics_context called with null pointer.");
    return;
  }

  let context = unsafe { &mut *(context_ptr as *mut SpringwaveContext) };
  context.advance(gametime);

  unsafe {
    *out_rb_sine_x = context.rigid_body_set[context.rb_sine].position().translation.x;
    *out_rb_sine_y = context.rigid_body_set[context.rb_sine].position().translation.y;
    *out_rb_link1_x = context.rigid_body_set[context.rb_link1].position().translation.x;
    *out_rb_link1_y = context.rigid_body_set[context.rb_link1].position().translation.y;
    *out_rb_link2_x = context.rigid_body_set[context.rb_link2].position().translation.x;
    *out_rb_link2_y = context.rigid_body_set[context.rb_link2].position().translation.y;
  }
}

#[no_mangle]
pub extern "C" fn destroy_springwave(context_ptr: *mut c_void) {
  if context_ptr.is_null() {
    return;
  }

  let _ = unsafe { Box::from_raw(context_ptr as *mut SpringwaveContext) };
}

#[cfg(test)]
mod tests {
  use super::*;

  #[test]
  fn test() {
    let mut gametime = 1300.0;
    let mut swc = SpringwaveContext::new(gametime);

    swc.advance(gametime + 1.0);
  }
}