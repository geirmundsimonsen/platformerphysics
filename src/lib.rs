
use rapier2d_f64::prelude::*;
use std::ffi::c_void;

mod test;
mod test_stream;
mod springwaves;

#[no_mangle]
pub extern "C" fn add(a: i32, b: i32) -> i32 {
  a + b
}

#[no_mangle]
pub extern "C" fn create_physics_context() -> *mut c_void {
    // Box the context onto the heap
    let context = Box::new(test_stream::PhysicsContext::new());
    // Return a raw pointer (leaking the Box's memory ownership to the caller)
    Box::into_raw(context) as *mut c_void
}

/// Steps the physics simulation for the given context.
/// `context_ptr` must be a valid pointer previously returned by `create_physics_context`.
#[no_mangle]
pub extern "C" fn step_physics_context(context_ptr: *mut c_void) {
    if context_ptr.is_null() {
        // Handle error: null pointer passed
        eprintln!("Error: step_physics_context called with null pointer.");
        return;
    }
    // SAFETY:
    // 1. We assume `context_ptr` is valid and points to a `PhysicsContext`
    //    allocated by `create_physics_context` and not yet destroyed.
    // 2. We assume this function is not called concurrently for the same `context_ptr`
    //    (unless `PhysicsContext` is internally thread-safe, which Rapier's core
    //    structures generally are not without external locking).
    // 3. We convert the raw pointer back to a mutable reference. This reference
    //    is only valid for the duration of this function call.
    let context = unsafe { &mut *(context_ptr as *mut test_stream::PhysicsContext) };
    context.step();
}

/// Destroys the physics context and frees its memory.
/// `context_ptr` must be a valid pointer previously returned by `create_physics_context`.
/// After calling this, `context_ptr` is invalid and must not be used again.
#[no_mangle]
pub extern "C" fn destroy_physics_context(context_ptr: *mut c_void) {
    if context_ptr.is_null() {
        // Optional: handle error or just do nothing for null.
        return;
    }
    // SAFETY:
    // 1. We assume `context_ptr` is valid and points to a `PhysicsContext`
    //    allocated by `create_physics_context`.
    // 2. This takes ownership of the pointer back from the C side.
    // 3. `Box::from_raw` reconstructs the Box.
    // 4. When the Box goes out of scope at the end of this function,
    //    it calls `drop` on `PhysicsContext`, freeing the memory.
    let _ = unsafe { Box::from_raw(context_ptr as *mut test_stream::PhysicsContext) };
    // The context is now dropped and memory freed.
}

// Example FFI function to get ball altitude
// Note: Passing handles as raw parts (index, generation) is common in FFI.
// You'll need a way for Unreal to know these handle parts.
#[no_mangle]
pub extern "C" fn get_body_translation_y(context_ptr: *mut c_void, handle_index: u32, handle_generation: u32, out_y: *mut f64) -> bool {
  if context_ptr.is_null() || out_y.is_null() {
        eprintln!("Error: get_body_y called with null pointer.");
      return false;
  }
  // SAFETY: See notes in `step_physics_context`. We need a shared reference here.
  let context = unsafe { &*(context_ptr as *mut test_stream::PhysicsContext) };
  // SAFETY: Ensure `out_y` points to valid memory writable by Rust.
  unsafe {
    if let Some(y) = context.get_body_translation_y(handle_index, handle_generation) {
      *out_y = y;
      true
    } else {
      false // Indicate body not found or handle invalid
    }
  }
}

#[cfg(test)]
mod tests {
  use super::*;

  #[ignore]
  #[test]
  fn test() {
    let mut pc = test_stream::PhysicsContext::new();

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