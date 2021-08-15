class_name Box2DWorld extends Node2D

var bodies: Array
var joints: Array
var arbiters: Dictionary

var gravity: Vector2
var iterations: int

var accumulate_impulses = true
var warm_starting = true
var position_correction = true


func add(p_object):
	if p_object is Box2DBody:
		bodies.push_back(p_object)
	elif p_object is Box2DJoint:
		joints.push_back(p_object)
	add_child(p_object)


func clear():
	for b in bodies:
		b.queue_free()
	bodies.clear()

	for j in joints:
		j.queue_free()
	joints.clear()

	arbiters.clear()


func broad_phase():
	# O(n^2) broad-phase.
	for i in range(0, bodies.size()):
		var bi = bodies[i]

		for j in range(i + 1, bodies.size()):
			var bj = bodies[j]

			if (bi.inv_mass == 0.0 && bj.inv_mass == 0.0):
				continue

			var new_arb = Box2DArbiter.new(bi, bj)
			var key = Box2DArbiter.Key.new(bi, bj).id

			if new_arb.num_contacts > 0:
				if !arbiters.has(key):
					arbiters[key] = new_arb
				else:
					arbiters[key].update(new_arb.contacts, new_arb.num_contacts)
			else:
				# warning-ignore:return_value_discarded
				arbiters.erase(key)


func step(dt: float):
	var inv_dt = 1.0 / dt if dt > 0.0 else 0.0

	# Determine overlapping bodies and update contact points.
	broad_phase();

	# Integrate forces.
	for i in bodies.size():
		var b = bodies[i]

		if b.inv_mass == 0.0:
			continue

		b.velocity += dt * (gravity + b.inv_mass * b.force)
		b.angular_velocity += dt * b.inv_I * b.torque

	# Perform pre-steps.
	for arb in arbiters.values():
		arb.pre_step(inv_dt)

	for i in joints.size():
		joints[i].pre_step(inv_dt);

	# Perform iterations.
	for i in iterations:
		for arb in arbiters.values():
			arb.apply_impulse()
		for j in joints.size():
			joints[j].apply_impulse()

	# Integrate velocities.
	for i in bodies.size():
		var b = bodies[i]

		b.position += dt * b.velocity
		b.rotation += dt * b.angular_velocity

		b.force = Vector2(0, 0)
		b.torque = 0.0
