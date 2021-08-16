class_name Box2DWorld extends Node2D

var bodies: Array
var joints: Array
var arbiters: Dictionary

export var gravity = Vector2(0, -10)
export var iterations = 10

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
		var body_a = bodies[i]

		for j in range(i + 1, bodies.size()):
			var body_b = bodies[j]

			if body_a.inv_mass == 0.0 and body_b.inv_mass == 0.0:
				continue

			var new_arb = Box2DArbiter.new(body_a, body_b) # Narrow-phase.
			var key = Box2DArbiter.Key.new(body_a, body_b).id

			if new_arb.num_contacts > 0:
				if not key in arbiters:
					arbiters[key] = new_arb
				else:
					arbiters[key].update(new_arb.contacts, new_arb.num_contacts)
			else:
				var _erased = arbiters.erase(key)


func step(dt: float):
	var inv_dt = 1.0 / dt if dt > 0.0 else 0.0

	# Determine overlapping bodies and update contact points.
	broad_phase()

	# Integrate forces.
	for body in bodies:
		if body.inv_mass == 0.0:
			continue
		body.velocity += dt * (gravity + body.inv_mass * body.force)
		body.angular_velocity += dt * body.inv_inertia * body.torque

	# Perform pre-steps.
	for arb in arbiters.values():
		arb.pre_step(inv_dt)

	for joint in joints:
		joint.pre_step(inv_dt)

	# Perform iterations.
	for i in iterations:
		for arb in arbiters.values():
			arb.apply_impulse()
		for joint in joints:
			joint.apply_impulse()

	# Integrate velocities.
	for body in bodies:
		body.position += dt * body.velocity
		body.rotation += dt * body.angular_velocity

		body.force = Vector2(0, 0)
		body.torque = 0.0
