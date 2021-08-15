class_name Box2DBody extends Node2D

var velocity: Vector2
var angular_velocity: float

var force: Vector2
var torque: float

var width: Vector2

var friction: float
var mass: float
var inv_mass: float
var I: float
var inv_I: float


func _init():
	position = Vector2()
	rotation = 0.0
	velocity = Vector2()
	angular_velocity = 0.0
	force = Vector2()
	torque = 0.0
	friction = 0.2

	width = Vector2(1, 1)
	mass = INF
	inv_mass = 0.0
	I = INF
	inv_I = 0.0


func add_force(p_force: Vector2):
	force += p_force


func setup(p_width: Vector2, p_mass: float):
	position = Vector2()
	rotation = 0.0
	velocity = Vector2()
	angular_velocity = 0.0
	force = Vector2()
	torque = 0.0
	friction = 0.2

	width = p_width
	mass = p_mass

	if mass < INF:
		inv_mass = 1.0 / mass
		I = mass * (width.x * width.x + width.y * width.y) / 12.0
		inv_I = 1.0 / I
	else:
		inv_mass = 0.0
		I = INF
		inv_I = 0.0


func _process(_delta):
	update()


func _draw():
	var h = 0.5 * width
	var line = [
		Vector2(-h.x, -h.y),
		Vector2( h.x, -h.y),
		Vector2( h.x,  h.y),
		Vector2(-h.x,  h.y),
	]
	for i in 4:
		draw_line(line[i], line[(i + 1) % 4], Color(0.8, 0.8, 0.9))
