class_name Box2DBody extends Node2D

var velocity = Vector2()
var angular_velocity = 0.0

var force = Vector2()
var torque = 0.0

var width = Vector2(1, 1)

var friction = 0.2
var mass = INF
var inv_mass = 0.0
var inertia = INF
var inv_inertia = 0.0


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
		inertia = mass * (width.x * width.x + width.y * width.y) / 12.0
		inv_inertia = 1.0 / inertia
	else:
		inv_mass = 0.0
		inertia = INF
		inv_inertia = 0.0


func _process(_delta):
	update()


func get_bounding_rect():
	var h = 0.5 * width
	var vertices = PoolVector2Array([
		Vector2(-h.x, -h.y),
		Vector2( h.x, -h.y),
		Vector2( h.x,  h.y),
		Vector2(-h.x,  h.y),
	])
	vertices = get_transform().xform(vertices)
	var rect
	for i in vertices.size():
		if i == 0:
			rect = Rect2(vertices[i], Vector2())
		else:
			rect = rect.expand(vertices[i])
	return rect


func _draw():
	var h = 0.5 * width
	var line = [
		Vector2(-h.x, -h.y),
		Vector2( h.x, -h.y),
		Vector2( h.x,  h.y),
		Vector2(-h.x,  h.y),
	]
	var c = Color(0.8, 0.8, 0.9)
	if has_meta("bomb"):
		c = Color(0.4, 0.9, 0.4)
	for i in 4:
		draw_line(line[i], line[(i + 1) % 4], c)
