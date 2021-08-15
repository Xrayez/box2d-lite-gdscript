extends Node2D

var bodies = []
var joints = []

var bomb: Box2DBody = null

var time_step = 1.0 / 60.0
var iterations = 10
var gravity = Vector2(0, -10)

var num_bodies = 0
var num_joints = 0

var demo_index = 0

var width = 1200
var height = 720

var zoom = 10.0
var pan_y = 8.0

var world

func _init():
	world = Box2DWorld.new()
	world.gravity = gravity
	world.iterations = iterations


func _ready():
	var b = Box2DBody.new()
	b.setup(Vector2(100, 20), INF)
	b.position = Vector2(0.0, -0.5 * b.width.y)
	world.add(b)
	add_child(b)

	b = Box2DBody.new()
	b.setup(Vector2(1, 1), 200)
	b.position = Vector2(0, 4)
	world.add(b)
	add_child(b)

	b = Box2DBody.new()
	b.setup(Vector2(1, 1), 200)
	b.position = Vector2(0.5, 2)
	world.add(b)
	add_child(b)


func _physics_process(delta):
	world.step(time_step)
