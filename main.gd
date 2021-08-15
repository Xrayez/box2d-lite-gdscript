extends Node2D

onready var world = $Box2DWorld
var bomb: Box2DBody = null

var time_step = 1.0 / 60.0
var iterations = 10
var gravity = Vector2(0, -10)

var demo_index = 1


func _ready():
	world.gravity = gravity
	world.iterations = iterations

	$Camera2D.offset_v = 0.1

	init_demo(1)


func init_demo(index: int):
	world.clear()
	bomb = null

	demo_index = index
	call("demo_%s" % demo_index)


func demo_1():
	var b = Box2DBody.new()
	b.setup(Vector2(100, 20), INF)
	b.position = Vector2(0.0, -0.5 * b.width.y)
	world.add(b)

	b = Box2DBody.new()
	b.setup(Vector2(1, 1), 200)
	b.position = Vector2(0, 4)
	world.add(b)


func _physics_process(_delta):
	world.step(time_step)
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

#				KEY_SPACE: launch_bomb()


func draw_text(p_text):
	$UI/Info.text += p_text + "\n\n"


func _draw():
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
	draw_text("(A)ccumulation %s" % "ON" if PhysicsWorld.accumulate_impulses else "OFF")
	draw_text("(P)osition Correction %s" % "ON" if PhysicsWorld.position_correction else "OFF")
	draw_text("(W)arm Starting %s" % "ON" if PhysicsWorld.warm_starting else "OFF")
