extends Node2D

onready var world = $Box2DWorld
var bomb: Box2DBody = null
var demo_index = 1


func _ready():
	init_demo(1)


func init_demo(index: int):
	world.clear()
	if is_instance_valid(bomb):
		bomb.queue_free()
		bomb = null

	$Camera2D.position = Vector2()
	$Camera2D.offset_v = 0.11

	demo_index = index
	call("demo_%s" % demo_index)


# Single box.
func demo_1():
	var b = Box2DBody.new()
	b.setup(Vector2(100, 20), INF)
	b.position = Vector2(0, -0.5 * b.width.y)
	world.add(b)

	b = Box2DBody.new()
	b.setup(Vector2(1, 1), 200)
	b.position = Vector2(0, 4)
	world.add(b)


# A simple pendulum.
func demo_2():
	var b1 = Box2DBody.new()
	b1.setup(Vector2(100, 20), INF)
	b1.friction = 0.2
	b1.position = Vector2(0, -0.5 * b1.width.y)
	b1.rotation = 0
	world.add(b1)

	var b2 = Box2DBody.new()
	b2.setup(Vector2(1, 1), 100)
	b2.friction = 0.2
	b2.position = Vector2(9.0, 11.0)
	b2.rotation = 0
	world.add(b2)

	var j = Box2DJoint.new()
	j.setup(b1, b2, Vector2(0, 11))
	world.add(j)


# Varying friction coefficients.
func demo_3():
	var b = Box2DBody.new()
	b.setup(Vector2(100.0, 20.0), INF)
	b.position = Vector2(0.0, -0.5 * b.width.y)
	world.add(b)

	b = Box2DBody.new()
	b.setup(Vector2(13.0, 0.25), INF)
	b.position = Vector2(-2.0, 11.0)
	b.rotation = -0.25
	world.add(b)

	b = Box2DBody.new()
	b.setup(Vector2(0.25, 1.0), INF)
	b.position = Vector2(5.25, 9.5)
	world.add(b)

	b = Box2DBody.new()
	b.setup(Vector2(13.0, 0.25), INF)
	b.position = Vector2(2.0, 7.0)
	b.rotation = 0.25
	world.add(b)

	b = Box2DBody.new()
	b.setup(Vector2(0.25, 1.0), INF)
	b.position = Vector2(-5.25, 5.5)
	world.add(b)

	b = Box2DBody.new()
	b.setup(Vector2(13.0, 0.25), INF)
	b.position = Vector2(-2.0, 3.0)
	b.rotation = -0.25
	world.add(b)

	var friction = [0.75, 0.5, 0.35, 0.1, 0.0]
	for i in 5:
		b = Box2DBody.new()
		b.setup(Vector2(0.5, 0.5), 25.0)
		b.friction = friction[i]
		b.position = Vector2(-7.5 + 2.0 * i, 14.0)
		world.add(b)


# A vertical stack.
func demo_4():
	var b = Box2DBody.new()
	b.setup(Vector2(100.0, 20.0), INF)
	b.friction = 0.2
	b.position = Vector2(0.0, -0.5 * b.width.y)
	b.rotation = 0.0
	world.add(b)

	for i in 10:
		b = Box2DBody.new()
		b.setup(Vector2(1, 1), 1)
		b.friction = 0.2
		var x = rand_range(-0.1, 0.1)
		b.position = Vector2(x, 0.51 + 1.05 * i)
		world.add(b)


# A pyramid.
func demo_5():
	var b = Box2DBody.new()
	b.setup(Vector2(100.0, 20.0), INF)
	b.friction = 0.2
	b.position = Vector2(0.0, -0.5 * b.width.y)
	b.rotation = 0.0
	world.add(b)

	var x = Vector2(-6.0, 0.75)
	var y

	for i in 12:
		y = x
		for _j in range(i, 12):
			b = Box2DBody.new()
			b.setup(Vector2(1.0, 1.0), 10.0)
			b.friction = 0.2
			b.position = y
			world.add(b)
			y += Vector2(1.125, 0.0)

#		x += Vector2(0.5625, 1.125)
		x += Vector2(0.5625, 2.0)


# A teeter.
func demo_6():
	var b1 = Box2DBody.new()
	b1.setup(Vector2(100.0, 20.0), INF)
	b1.position = Vector2(0.0, -0.5 * b1.width.y)
	world.add(b1)

	var b2 = Box2DBody.new()
	b2.setup(Vector2(12.0, 0.25), 100.0)
	b2.position = Vector2(0.0, 1.0)
	world.add(b2)

	var b3 = Box2DBody.new()
	b3.setup(Vector2(0.5, 0.5), 25.0)
	b3.position = Vector2(-5.0, 2.0)
	world.add(b3)

	var b4 = Box2DBody.new()
	b4.setup(Vector2(0.5, 0.5), 25.0)
	b4.position = Vector2(-5.5, 2.0)
	world.add(b4)

	var b5 = Box2DBody.new()
	b5.setup(Vector2(1.0, 1.0), 100.0)
	b5.position = Vector2(5.5, 15.0)
	world.add(b5)

	var j = Box2DJoint.new()
	j.setup(b1, b2, Vector2(0.0, 1.0))
	world.add(j)


# A suspension bridge.
func demo_7():
	var b = Box2DBody.new()
	b.setup(Vector2(100.0, 20.0), INF)
	b.friction = 0.2
	b.position = Vector2(0.0, -0.5 * b.width.y)
	b.rotation = 0.0
	world.add(b)

	var num_planks = 15
	var mass = 50.0

	for i in num_planks:
		b = Box2DBody.new()
		b.setup(Vector2(1.0, 0.25), mass)
		b.friction = 0.2
		b.position = Vector2(-8.5 + 1.25 * i, 5.0)
		world.add(b)

	# Tuning.
	var frequency_hz = 2.0
	var damping_ratio = 0.7

	# Frequency in radians.
	var omega = 2.0 * PI * frequency_hz

	# Damping coefficient.
	var d = 2.0 * mass * damping_ratio * omega

	# Spring stifness.
	var k = mass * omega * omega

	# Magic formulas.
	var time_step = get_physics_process_delta_time()
	var softness = 1.0 / (d + time_step * k)
	var bias_factor = time_step * k / (d + time_step * k)

	for i in num_planks:
		var j = Box2DJoint.new()
		j.setup(world.bodies[i], world.bodies[i + 1], Vector2(-9.125 + 1.25 * i, 5.0))
		j.softness = softness
		j.bias_factor = bias_factor
		world.add(j)

	var j = Box2DJoint.new()
	j.setup(world.bodies[num_planks], world.bodies[0], Vector2(-9.125 + 1.25 * num_planks, 5.0))
	j.softness = softness
	j.bias_factor = bias_factor
	world.add(j)


# Dominos.
func demo_8():
	var b = Box2DBody.new()
	var b1 = b
	b.setup(Vector2(100.0, 20.0), INF)
	b.position = Vector2(0.0, -0.5 * b.width.y)
	world.add(b)

	b = Box2DBody.new()
	b.setup(Vector2(12.0, 0.5), INF)
	b.position = Vector2(-1.5, 10.0)
	world.add(b)

	for i in 10:
		b = Box2DBody.new()
		b.setup(Vector2(0.2, 2.0), 10.0)
		b.position = Vector2(-6.0 + 1.0 * i, 11.125)
		b.friction = 0.1
		world.add(b)

	b = Box2DBody.new()
	b.setup(Vector2(14.0, 0.5), INF)
	b.position = Vector2(1.0, 6.0)
	b.rotation = 0.3
	world.add(b)

	b = Box2DBody.new()
	var b2 = b
	b.setup(Vector2(0.5, 3.0), INF)
	b.position = Vector2(-7.0, 4.0)
	world.add(b)

	b = Box2DBody.new()
	var b3 = b
	b.setup(Vector2(12.0, 0.25), 20.0)
	b.position = Vector2(-0.9, 1.0)
	world.add(b)

	var j = Box2DJoint.new()
	j.setup(b1, b3, Vector2(-2.0, 1.0))
	world.add(j)

	b = Box2DBody.new()
	var b4 = b
	b.setup(Vector2(0.5, 0.5), 10.0)
	b.position = Vector2(-10.0, 15.0)
	world.add(b)

	j = Box2DJoint.new()
	j.setup(b2, b4, Vector2(-7.0, 15.0))
	world.add(j)

	b = Box2DBody.new()
	var b5 = b
	b.setup(Vector2(2.0, 2.0), 20.0)
	b.position = Vector2(6.0, 2.5)
	b.friction = 0.1
	world.add(b)

	j = Box2DJoint.new()
	j.setup(b1, b5, Vector2(6.0, 2.6))
	world.add(j)

	b = Box2DBody.new()
	var b6 = b
	b.setup(Vector2(2.0, 0.2), 10.0)
	b.position = Vector2(6.0, 3.6)
	world.add(b)

	j = Box2DJoint.new()
	j.setup(b5, b6, Vector2(7.0, 3.5))
	world.add(j)


# A multi-pendulum.
func demo_9():
	var b = Box2DBody.new()
	b.setup(Vector2(100.0, 20.0), INF)
	b.friction = 0.2
	b.position = Vector2(0.0, -0.5 * b.width.y)
	b.rotation = 0.0
	world.add(b)

	var b1 = b

	var mass = 10.0

	# Tuning.
	var frequency_hz = 4.0
	var damping_ratio = 0.7

	# Frequency in radians.
	var omega = 2.0 * PI * frequency_hz

	# Damping coefficient.
	var d = 2.0 * mass * damping_ratio * omega

	# Spring stiffness.
	var k = mass * omega * omega

	# Magic formulas.
	var time_step = get_physics_process_delta_time()
	var softness = 1.0 / (d + time_step * k)
	var bias_factor = time_step * k / (d + time_step * k)

	var y = 12.0

	for i in 15:
		var x = Vector2(0.5 + i, y)
		b = Box2DBody.new()
		b.setup(Vector2(0.75, 0.25), mass)
		b.friction = 0.2
		b.position = x
		b.rotation = 0.0
		world.add(b)

		var j = Box2DJoint.new()
		j.setup(b1, b, Vector2(float(i), y))
		j.softness = softness
		j.bias_factor = bias_factor
		world.add(j)

		b1 = b


func launch_bomb():
	if not is_instance_valid(bomb):
		bomb = Box2DBody.new()
		bomb.setup(Vector2(1, 1), 50)
		bomb.friction = 0.2
		bomb.set_meta("bomb", true)
		world.add(bomb)

	bomb.position = Vector2(rand_range(-15, 15), 15)
	bomb.rotation = rand_range(-1.5, 1.5)
	bomb.velocity = -1.5 * bomb.position
	bomb.angular_velocity = rand_range(-20, 20)


# Process as fast as possible, but use _physics_process() delta time instead.
# This emulates Box2D Lite's main loop.
func _process(_delta):
	world.step(get_physics_process_delta_time())
	update()


func _input(event):
	if event is InputEventKey:
		if event.pressed and not event.echo:
			match event.scancode:
				KEY_1: init_demo(1)
				KEY_2: init_demo(2)
				KEY_3: init_demo(3)
				KEY_4: init_demo(4)
				KEY_5: init_demo(5)
				KEY_6: init_demo(6)
				KEY_7: init_demo(7)
				KEY_8: init_demo(8)
				KEY_9: init_demo(9)

				KEY_A: PhysicsWorld.accumulate_impulses = not PhysicsWorld.accumulate_impulses
				KEY_P: PhysicsWorld.position_correction = not PhysicsWorld.position_correction
				KEY_W: PhysicsWorld.warm_starting = not PhysicsWorld.warm_starting

				KEY_SPACE: launch_bomb()


func draw_text(p_text):
	$UI/Info.text += p_text + "\n\n"


func draw_info_panel():
	$UI/Info.text = ""

	var demo_str = [
		"Demo 1: A Single Box",
		"Demo 2: Simple Pendulum",
		"Demo 3: Varying Friction Coefficients",
		"Demo 4: Randomized Stacking",
		"Demo 5: Pyramid Stacking",
		"Demo 6: A Teeter",
		"Demo 7: A Suspension Bridge",
		"Demo 8: Dominos",
		"Demo 9: Multi-pendulum",
	][demo_index - 1]

	draw_text(demo_str)
	draw_text("Keys: 1-9 Demos, Space to Launch the Bomb")
	draw_text("(A)ccumulation %s" % ("ON" if PhysicsWorld.accumulate_impulses else "OFF"))
	draw_text("(P)osition Correction %s" % ("ON" if PhysicsWorld.position_correction else "OFF"))
	draw_text("(W)arm Starting %s" % ("ON" if PhysicsWorld.warm_starting else "OFF"))


func draw_contact_points():
	for arb in world.arbiters.values():
		for i in arb.num_contacts:
			var c = Color.red
			c.s = 0.85
			draw_circle(arb.contacts[i].position, 0.075, c)


func _draw():
	draw_info_panel()
	draw_contact_points()
