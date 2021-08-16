class_name Box2DArbiter

class FeaturePair:
	var e = {
		in_edge_1 = 0,
		out_edge_1 = 0,
		in_edge_2 = 0,
		out_edge_2 = 0,
	}
	var value := 0

class Contact:
	var position: Vector2
	var normal: Vector2
	var r1: Vector2
	var r2: Vector2
	var separation: float
	var pn: float = 0.0 # Accumulated normal impulse.
	var pt: float = 0.0 # Accumulated tangent impulse.
	var pnb: float = 0.0 # Accumulated normal impulse for position bias.
	var mass_normal: float
	var mass_tangent: float
	var bias: float
	var feature: FeaturePair

class Key:
	var id

	func _init(b1, b2):
		if b1.get_instance_id() < b2.get_instance_id():
			id = [b1.get_instance_id(), b2.get_instance_id()]
		else:
			id = [b2.get_instance_id(), b1.get_instance_id()]

# Arbiter

const MAX_POINTS = 2

var contacts = []
var num_contacts: int

var body_1: Box2DBody
var body_2: Box2DBody

var friction: float # Combined friction.


func _init(b1: Box2DBody, b2: Box2DBody):
	contacts.push_back(Contact.new())
	contacts.push_back(Contact.new())

	if b1.get_instance_id() < b2.get_instance_id():
		body_1 = b1
		body_2 = b2
	else:
		body_1 = b2
		body_2 = b1

	num_contacts = collide(contacts, body_1, body_2)

	friction = sqrt(body_1.friction * body_2.friction)


func update(new_contacts: Array, num_new_contacts: int):
	var merged_contacts = []
	merged_contacts.push_back(Contact.new())
	merged_contacts.push_back(Contact.new())

	for i in num_new_contacts:
		var c_new = new_contacts[i]
		var k = -1
		for j in num_contacts:
			var c_old = contacts[j]
			if c_new.feature.value == c_old.feature.value:
				k = j
				break

		if k > -1:
			var c = merged_contacts[i]
			var c_old = contacts[k]
			merged_contacts[i] = c_new
			if PhysicsWorld.warm_starting:
				c.pn = c_old.pn
				c.pt = c_old.pt
				c.pnb = c_old.pnb
			else:
				c.pn = 0.0
				c.pt = 0.0
				c.pnb = 0.0
		else:
			merged_contacts[i] = new_contacts[i]

	for i in num_new_contacts:
		contacts[i] = merged_contacts[i]

	num_contacts = num_new_contacts


func pre_step(inv_dt: float):
	var k_allowed_penetration = 0.01
	var k_bias_factor = 0.2 if PhysicsWorld.position_correction else 0.0

	for i in num_contacts:
		var c = contacts[i]

		var r1 = c.position - body_1.position
		var r2 = c.position - body_2.position

		# Precompute normal mass, tangent mass, and bias.
		var rn1 = r1.dot(c.normal)
		var rn2 = r2.dot(c.normal)
		var k_normal = body_1.inv_mass + body_2.inv_mass
		k_normal += body_1.inv_inertia * (r1.dot(r1) - rn1 * rn1) + body_2.inv_inertia * (r2.dot(r2) - rn2 * rn2)
		c.mass_normal = 1.0 / k_normal

		var tangent = Math.cross_scalar(c.normal, 1.0)
		var rt1 = r1.dot(tangent)
		var rt2 = r2.dot(tangent)
		var k_tangent = body_1.inv_mass + body_2.inv_mass
		k_tangent += body_1.inv_inertia * (r1.dot(r1) - rt1 * rt1) + body_2.inv_inertia * (r2.dot(r2) - rt2 * rt2)
		c.mass_tangent = 1.0 /  k_tangent

		c.bias = -k_bias_factor * inv_dt * min(0.0, c.separation + k_allowed_penetration)

		if PhysicsWorld.accumulate_impulses:
			# Apply normal + friction impulse
			var P = c.pn * c.normal + c.pt * tangent

			body_1.velocity -= body_1.inv_mass * P
			body_1.angular_velocity -= body_1.inv_inertia * r1.cross(P)

			body_2.velocity += body_2.inv_mass * P
			body_2.angular_velocity += body_2.inv_inertia * r2.cross(P)


func apply_impulse():
	var b1 = body_1
	var b2 = body_2

	for i in num_contacts:
		var c = contacts[i]
		c.r1 = c.position - b1.position
		c.r2 = c.position - b2.position

		# Relative velocity at contact.
		var dv = b2.velocity + Math.cross_vector(b2.angular_velocity, c.r2) - b1.velocity - Math.cross_vector(b1.angular_velocity, c.r1)

		# Compute normal impulse.
		var vn = dv.dot(c.normal)

		var dpn = c.mass_normal * (-vn + c.bias)

		if PhysicsWorld.accumulate_impulses:
			# Clamp the accumulated impulse.
			var pn0 = c.pn
			c.pn = max(pn0 + dpn, 0.0)
			dpn = c.pn - pn0
		else:
			dpn = max(dpn, 0.0)

		# Apply contact impulse.
		var pn = dpn * c.normal

		b1.velocity -= b1.inv_mass * pn
		b1.angular_velocity -= b1.inv_inertia * c.r1.cross(pn)

		b2.velocity += b2.inv_mass * pn
		b2.angular_velocity += b2.inv_inertia * c.r2.cross(pn)

		# Relative velocity at contact.
		dv = b2.velocity + Math.cross_vector(b2.angular_velocity, c.r2) - b1.velocity - Math.cross_vector(b1.angular_velocity, c.r1)

		var tangent = Math.cross_scalar(c.normal, 1.0)
		var vt = dv.dot(tangent)
		var dpt = c.mass_tangent * (-vt)

		if PhysicsWorld.accumulate_impulses:
			# Compute friction impulse.
			var max_pt = friction * c.pn

			# Clamp friction.
			var old_tangent_impulse = c.pt
			c.pt = clamp(old_tangent_impulse + dpt, -max_pt, max_pt)
			dpt = c.pt - old_tangent_impulse
		else:
			var max_pt = friction * dpn
			dpt = clamp(dpt, -max_pt, max_pt)

		# Apply contact impulse
		var pt = dpt * tangent

		b1.velocity -= b1.inv_mass * pt
		b1.angular_velocity -= b1.inv_inertia * c.r1.cross(pt)

		b2.velocity += b2.inv_mass * pt
		b2.angular_velocity += b2.inv_inertia * c.r2.cross(pt)

# Collide

# Box vertex and edge numbering:
#
#        ^ y
#        |
#        e1
#   v2 ------ v1
#    |        |
# e2 |        | e4  -. x
#    |        |
#   v3 ------ v4
#        e3

enum Axis {
	FACE_A_X,
	FACE_A_Y,
	FACE_B_X,
	FACE_B_Y,
}

enum EdgeNumbers {
	NO_EDGE = 0,
	EDGE_1,
	EDGE_2,
	EDGE_3,
	EDGE_4,
}

class ClipVertex:
	func _init():
		fp.value = 0
	var v: Vector2
	var fp = FeaturePair.new()


static func flip(fp):
	var t1 = fp.e.in_edge_1
	fp.e.in_edge_1 = fp.e.in_edge_2
	fp.e.in_edge_2 = t1

	var t2 = fp.e.out_edge_1
	fp.e.out_edge_1 = fp.e.out_edge_2
	fp.e.out_edge_2 = t2


static func clip_segment_to_line(v_out: Array, v_in: Array, normal: Vector2, offset: float, clip_edge: int):
	# Start with no output points.
	var num_out = 0

	# Calculate the distance of end points to the line.
	var distance_0 = normal.dot(v_in[0].v) - offset
	var distance_1 = normal.dot(v_in[1].v) - offset

	# If the points are behind the plane.
	if distance_0 <= 0.0:
		v_out[num_out] = v_in[0]
		num_out += 1
	if distance_1 <= 0.0:
		v_out[num_out] = v_in[1]
		num_out += 1

	# If the points are on different sides of the plane.
	if distance_0 * distance_1 < 0.0:
		# Find intersection point of edge and plane.
		var interp = distance_0 / (distance_0 - distance_1)
		v_out[num_out].v = v_in[0].v + interp * (v_in[1].v - v_in[0].v)

		if distance_0 > 0.0:
			v_out[num_out].fp = v_in[0].fp
			v_out[num_out].fp.e.in_edge_1 = clip_edge
			v_out[num_out].fp.e.in_edge_2 = EdgeNumbers.NO_EDGE
		else:
			v_out[num_out].fp = v_in[1].fp
			v_out[num_out].fp.e.out_edge_1 = clip_edge
			v_out[num_out].fp.e.out_edge_2 = EdgeNumbers.NO_EDGE

		num_out += 1

	return num_out


static func compute_incident_edge(c: Array, h: Vector2, pos: Vector2, Rot: Transform2D, normal: Vector2):
	# The normal is from the reference box. Convert it
	# to the incident boxe's frame and flip sign.
	var RotT = Rot.inverse()
	var n = -(RotT * normal)
	var n_abs = n.abs()

	if n_abs.x > n_abs.y:
		if sign(n.x) > 0:
			c[0].v = Vector2(h.x, -h.y)
			c[0].fp.e.in_edge_2 = EdgeNumbers.EDGE_3
			c[0].fp.e.out_edge_2 = EdgeNumbers.EDGE_4

			c[1].v = Vector2(h.x, h.y)
			c[1].fp.e.in_edge_2 = EdgeNumbers.EDGE_4
			c[1].fp.e.out_edge_2 = EdgeNumbers.EDGE_1
		else:
			c[0].v = Vector2(-h.x, h.y)
			c[0].fp.e.in_edge_2 = EdgeNumbers.EDGE_1
			c[0].fp.e.out_edge_2 = EdgeNumbers.EDGE_2

			c[1].v = Vector2(-h.x, -h.y)
			c[1].fp.e.in_edge_2 = EdgeNumbers.EDGE_2
			c[1].fp.e.out_edge_2 = EdgeNumbers.EDGE_3
	else:
		if sign(n.y) > 0:
			c[0].v = Vector2(h.x, h.y)
			c[0].fp.e.in_edge_2 = EdgeNumbers.EDGE_4
			c[0].fp.e.out_edge_2 = EdgeNumbers.EDGE_1

			c[1].v = Vector2(-h.x, h.y)
			c[1].fp.e.in_edge_2 = EdgeNumbers.EDGE_1
			c[1].fp.e.out_edge_2 = EdgeNumbers.EDGE_2
		else:
			c[0].v = Vector2(-h.x, -h.y)
			c[0].fp.e.in_edge_2 = EdgeNumbers.EDGE_2
			c[0].fp.e.out_edge_2 = EdgeNumbers.EDGE_3

			c[1].v = Vector2(h.x, -h.y)
			c[1].fp.e.in_edge_2 = EdgeNumbers.EDGE_3
			c[1].fp.e.out_edge_2 = EdgeNumbers.EDGE_4

	c[0].v = pos + Rot * c[0].v
	c[1].v = pos + Rot * c[1].v


# The normal points from A to B.
static func collide(p_contacts, body_a, body_b):
	# Setup.
	var h_a = 0.5 * body_a.width
	var h_b = 0.5 * body_b.width

	var pos_a = body_a.position
	var pos_b = body_b.position

	var rot_a = Transform2D(body_a.rotation, Vector2())
	var rot_b = Transform2D(body_b.rotation, Vector2())

	var rot_at = rot_a.inverse()
	var rot_bt = rot_b.inverse()

	var dp = pos_b - pos_a
	var d_a = rot_at * dp
	var d_b = rot_bt * dp

	var c = rot_at * rot_b
	var abs_c = Transform2D()
	abs_c.x = c.x.abs()
	abs_c.y = c.y.abs()
	var abs_ct = abs_c.inverse()

	# Box A faces.
	var face_a = d_a.abs() - h_a - abs_c * h_b
	if face_a.x > 0.0 or face_a.y > 0.0:
		return 0

	# Box B faces.
	var face_b = d_b.abs() - abs_ct * h_a - h_b
	if face_b.x > 0.0 or face_b.y > 0.0:
		return 0

	# Box A faces.
	var axis = Axis.FACE_A_X
	var separation = face_a.x
	var normal = rot_a.x if d_a.x > 0.0 else -rot_a.x

	var relative_tol = 0.95
	var absolute_tol = 0.01

	if face_a.y > relative_tol * separation + absolute_tol * h_a.y:
		axis = Axis.FACE_A_Y
		separation = face_a.y
		normal = rot_a.y if d_a.y > 0.0 else -rot_a.y

	# Box B faces.
	if face_b.x > relative_tol * separation + absolute_tol * h_b.x:
		axis = Axis.FACE_B_X
		separation = face_b.x
		normal = rot_b.x if d_b.x > 0.0 else -rot_b.x

	if face_b.y > relative_tol * separation + absolute_tol * h_b.y:
		axis = Axis.FACE_B_Y
		separation = face_b.y
		normal = rot_b.y if d_b.y > 0.0 else -rot_b.y

	# Setup clipping plane data based on the separating axis.
	var front_normal
	var side_normal
	var incident_edge = []
	incident_edge.push_back(ClipVertex.new())
	incident_edge.push_back(ClipVertex.new())

	var front
	var neg_side
	var pos_side
	var neg_edge
	var pos_edge

	# Compute the clipping lines and the line segment to be clipped..
	match axis:
		Axis.FACE_A_X:
			front_normal = normal
			front = pos_a.dot(front_normal) + h_a.x
			side_normal = rot_a.y
			var side = pos_a.dot(side_normal)
			neg_side = -side + h_a.y
			pos_side =  side + h_a.y
			neg_edge = EdgeNumbers.EDGE_3
			pos_edge = EdgeNumbers.EDGE_1
			compute_incident_edge(incident_edge, h_b, pos_b, rot_b, front_normal)
		Axis.FACE_A_Y:
			front_normal = normal
			front = pos_a.dot(front_normal) + h_a.y
			side_normal = rot_a.x
			var side = pos_a.dot(side_normal)
			neg_side = -side + h_a.x
			pos_side =  side + h_a.x
			neg_edge = EdgeNumbers.EDGE_2
			pos_edge = EdgeNumbers.EDGE_4
			compute_incident_edge(incident_edge, h_b, pos_b, rot_b, front_normal)
		Axis.FACE_B_X:
			front_normal = -normal
			front = pos_b.dot(front_normal) + h_b.x
			side_normal = rot_b.y
			var side = pos_b.dot(side_normal)
			neg_side = -side + h_b.y
			pos_side =  side + h_b.y
			neg_edge = EdgeNumbers.EDGE_3
			pos_edge = EdgeNumbers.EDGE_1
			compute_incident_edge(incident_edge, h_a, pos_a, rot_a, front_normal)
		Axis.FACE_B_Y:
			front_normal = -normal
			front = pos_b.dot(front_normal) + h_b.y
			side_normal = rot_b.x
			var side = pos_b.dot(side_normal)
			neg_side = -side + h_b.x
			pos_side =  side + h_b.x
			neg_edge = EdgeNumbers.EDGE_2
			pos_edge = EdgeNumbers.EDGE_4
			compute_incident_edge(incident_edge, h_a, pos_a, rot_a, front_normal)

	# Clip other face with 5 box planes (1 face plane, 4 edge planes).
	var clip_points_1 = []
	clip_points_1.push_back(ClipVertex.new())
	clip_points_1.push_back(ClipVertex.new())
	var clip_points_2 = []
	clip_points_2.push_back(ClipVertex.new())
	clip_points_2.push_back(ClipVertex.new())

	# Clip to box side 1.
	var np = clip_segment_to_line(clip_points_1, incident_edge, -side_normal, neg_side, neg_edge)
	if np < 2:
		return 0

	# Clip to negative box side 1.
	np = clip_segment_to_line(clip_points_2, clip_points_1,  side_normal, pos_side, pos_edge)
	if np < 2:
		return 0

	# Now clip_points_2 contains the clipping points..
	# Due to roundoff, it is possible that clipping removes all points..
	var count = 0
	for i in 2:
		var sep = front_normal.dot(clip_points_2[i].v) - front
		if sep <= 0:
			p_contacts[count].separation = sep
			p_contacts[count].normal = normal
			# Slide contact point onto reference face (easy to cull).
			p_contacts[count].position = clip_points_2[i].v - sep * front_normal
			p_contacts[count].feature = clip_points_2[i].fp
			if axis == Axis.FACE_B_X or axis == Axis.FACE_B_Y:
				flip(p_contacts[count].feature)
			count += 1

	return count
