class_name Math


static func cross_scalar(v: Vector2, s: float):
	return Vector2(s * v.y, -s * v.x)


static func cross_vector(s: float, v: Vector2):
	return Vector2(-s * v.y, s * v.x)
