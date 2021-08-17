class_name Box2DJoint extends Node2D

var m := Transform2D()
var local_anchor_1: Vector2
var local_anchor_2: Vector2
var r1: Vector2
var r2: Vector2
var bias: Vector2
var p = Vector2()  # Accumulated impulse.
var body_1 = null
var body_2 = null
var bias_factor = 0.2
var softness = 0.0


func setup(p_body_1, p_body_2, p_anchor: Vector2):
	body_1 = p_body_1
	body_2 = p_body_2

	var rot_1 = Transform2D(body_1.rotation, Vector2())
	var rot_2 = Transform2D(body_2.rotation, Vector2())

	local_anchor_1 = rot_1.inverse() * (p_anchor - body_1.position)
	local_anchor_2 = rot_2.inverse() * (p_anchor - body_2.position)

	p = Vector2()

	softness = 0.0
	bias_factor = 0.2


func pre_step(inv_dt: float):
	# Pre-compute anchors, mass matrix, and bias.
	var rot_1 = Transform2D(body_1.rotation, Vector2())
	var rot_2 = Transform2D(body_2.rotation, Vector2())

	r1 = rot_1 * local_anchor_1
	r2 = rot_2 * local_anchor_2

	# deltaV = deltaV0 + k * impulse
	# invM = [(1/m1 + 1/m2) * eye(2) - skew(r1) * inv_inertia1 * skew(r1) - skew(r2) * inv_inertia2 * skew(r2)]
	#      = [1/m1+1/m2     0    ] + inv_inertia1 * [r1.y*r1.y -r1.x*r1.y] + inv_inertia2 * [r1.y*r1.y -r1.x*r1.y]
	#        [    0     1/m1+1/m2]            [-r1.x*r1.y r1.x*r1.x]            [-r1.x*r1.y r1.x*r1.x]
	var k1 := Transform2D()
	k1.x.x = body_1.inv_mass + body_2.inv_mass;   k1.y.x = 0.0;
	k1.x.y = 0.0;                                 k1.y.y = body_1.inv_mass + body_2.inv_mass;

	var k2 := Transform2D()
	k2.x.x =  body_1.inv_inertia * r1.y * r1.y;   k2.y.x = -body_1.inv_inertia * r1.x * r1.y;
	k2.x.y = -body_1.inv_inertia * r1.x * r1.y;   k2.y.y =  body_1.inv_inertia * r1.x * r1.x;

	var k3 := Transform2D()
	k3.x.x =  body_2.inv_inertia * r2.y * r2.y;   k3.y.x = -body_2.inv_inertia * r2.x * r2.y;
	k3.x.y = -body_2.inv_inertia * r2.x * r2.y;   k3.y.y =  body_2.inv_inertia * r2.x * r2.x;

	var k := Transform2D()
	k.x = k1.x + k2.x + k3.x
	k.y = k1.y + k2.y + k3.y
	k.x.x += softness
	k.y.y += softness

	m = k.affine_inverse()

	var p1 = body_1.position + r1
	var p2 = body_2.position + r2
	var dp = p2 - p1

	if PhysicsWorld.accumulate_impulses:
		bias = -bias_factor * inv_dt * dp
	else:
		bias = Vector2()

	if PhysicsWorld.warm_starting:
		# Apply accumulated impulse.
		body_1.velocity -= body_1.inv_mass * p
		body_1.angular_velocity -= body_1.inv_inertia * r1.cross(p)

		body_2.velocity += body_2.inv_mass * p
		body_2.angular_velocity += body_2.inv_inertia * r2.cross(p)
	else:
		p = Vector2()


func apply_impulse():
	# Relative velocity at contact.
	var cr_1 = Vector2(-body_1.angular_velocity * r1.y, body_1.angular_velocity * r1.x)
	var cr_2 = Vector2(-body_2.angular_velocity * r2.y, body_2.angular_velocity * r2.x)
	var dv = body_2.velocity + cr_2 - body_1.velocity - cr_1

	var impulse = m * (bias - dv - softness * p)

	body_1.velocity -= body_1.inv_mass * impulse
	body_1.angular_velocity -= body_1.inv_inertia * r1.cross(impulse)

	body_2.velocity += body_2.inv_mass * impulse
	body_2.angular_velocity += body_2.inv_inertia * r2.cross(impulse)

	p += impulse


func _process(_delta):
	update()


func _draw():
	var x1 = body_1.position
	var p1 = x1 + Transform2D(body_1.rotation, Vector2()) * local_anchor_1

	var x2 = body_2.position
	var p2 = x2 + Transform2D(body_2.rotation, Vector2()) * local_anchor_2

	var lines = [
		Vector2(x1.x, x1.y),
		Vector2(p1.x, p1.y),
		Vector2(x2.x, x2.y),
		Vector2(p2.x, p2.y),
	]
	for i in 3:
		draw_line(lines[i], lines[i + 1], Color(0.5, 0.5, 0.8))
