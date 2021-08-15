class_name Box2DJoint extends Node2D

var M := Transform2D()
var local_anchor_1: Vector2
var local_anchor_2: Vector2
var r1: Vector2
var r2: Vector2
var bias: Vector2
var P: Vector2  # Accumulated impulse.
var body_1
var body_2
var bias_factor: float
var softness: float


func _init():
	body_1 = null
	body_2 = null
	P = Vector2()
	bias_factor = 0.2
	softness = 0.0


func setup(p_body_1, p_body_2, p_anchor: Vector2):
	body_1 = p_body_1
	body_2 = p_body_2

	var Rot1 = Transform2D(body_1.rotation, Vector2())
	var Rot2 = Transform2D(body_2.rotation, Vector2())
	var Rot1T = Rot1.inverse()
	var Rot2T = Rot2.inverse()

	local_anchor_1 = Rot1T * (p_anchor - body_1.position)
	local_anchor_2 = Rot2T * (p_anchor - body_2.position)

	P = Vector2()

	softness = 0.0
	bias_factor = 0.2


func pre_step(inv_dt: float):
	# Pre-compute anchors, mass matrix, and bias.
	var Rot1 = Transform2D(body_1.rotation, Vector2())
	var Rot2 = Transform2D(body_2.rotation, Vector2())

	r1 = Rot1 * local_anchor_1
	r2 = Rot2 * local_anchor_2

	# deltaV = deltaV0 + K * impulse
	# invM = [(1/m1 + 1/m2) * eye(2) - skew(r1) * inv_I1 * skew(r1) - skew(r2) * inv_I2 * skew(r2)]
	#      = [1/m1+1/m2     0    ] + inv_I1 * [r1.y*r1.y -r1.x*r1.y] + inv_I2 * [r1.y*r1.y -r1.x*r1.y]
	#        [    0     1/m1+1/m2]            [-r1.x*r1.y r1.x*r1.x]            [-r1.x*r1.y r1.x*r1.x]
	var K1 := Transform2D()
	K1.x.x = body_1.inv_mass + body_2.inv_mass;  K1.y.x = 0.0;
	K1.x.y = 0.0;                                K1.y.y = body_1.inv_mass + body_2.inv_mass;

	var K2 := Transform2D()
	K2.x.x =  body_1.inv_I * r1.y * r1.y;        K2.y.x = -body_1.inv_I * r1.x * r1.y;
	K2.x.y = -body_1.inv_I * r1.x * r1.y;        K2.y.y =  body_1.inv_I * r1.x * r1.x;

	var K3 := Transform2D()
	K3.x.x =  body_2.inv_I * r2.y * r2.y;        K3.y.x = -body_2.inv_I * r2.x * r2.y;
	K3.x.y = -body_2.inv_I * r2.x * r2.y;        K3.y.y =  body_2.inv_I * r2.x * r2.x;

	var K := Transform2D()
	K.x = K1.x + K2.x + K3.x
	K.y = K1.y + K2.y + K3.y
	K.x.x += softness
	K.y.y += softness

	M = K.affine_inverse()

	var p1 = body_1.position + r1
	var p2 = body_2.position + r2
	var dp = p2 - p1

	if PhysicsWorld.accumulate_impulses:
		bias = -bias_factor * inv_dt * dp
	else:
		bias = Vector2()

	if PhysicsWorld.warm_starting:
		# Apply accumulated impulse.
		body_1.velocity -= body_1.inv_mass * P
		body_1.angular_velocity -= body_1.inv_I * r1.cross(P)

		body_2.velocity += body_2.inv_mass * P
		body_2.angular_velocity += body_2.inv_I * r2.cross(P)
	else:
		P = Vector2()


func apply_impulse():
	var dv = body_2.velocity + Math.cross_vector(body_2.angular_velocity, r2) - body_1.velocity - Math.cross_vector(body_1.angular_velocity, r1)

	var impulse = M * (bias - dv - softness * P)

	body_1.velocity -= body_1.inv_mass * impulse
	body_1.angular_velocity -= body_1.inv_I * r1.cross(impulse)

	body_2.velocity += body_2.inv_mass * impulse
	body_2.angular_velocity += body_2.inv_I * r2.cross(impulse)

	P += impulse


func _process(_delta):
	update()


func _draw():
	var R1 = Transform2D(body_1.rotation, Vector2())
	var R2 = Transform2D(body_2.rotation, Vector2())

	var x1 = body_1.position
	var p1 = x1 + R1 * local_anchor_1

	var x2 = body_2.position
	var p2 = x2 + R2 * local_anchor_2

	var lines = [
		Vector2(x1.x, x1.y),
		Vector2(p1.x, p1.y),
		Vector2(x2.x, x2.y),
		Vector2(p2.x, p2.y),
	]
	for i in 3:
		draw_line(lines[i], lines[i + 1], Color(0.5, 0.5, 0.8))
