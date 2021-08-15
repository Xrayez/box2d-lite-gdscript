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
	var Pn: float = 0.0 # Accumulated normal impulse.
	var Pt: float = 0.0 # Accumulated tangent impulse.
	var Pnb: float = 0.0 # Accumulated normal impulse for position bias.
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

		if (k > -1):
			var c = merged_contacts[i]
			var c_old = contacts[k]
			merged_contacts[i] = c_new
			if PhysicsWorld.warm_starting:
				c.Pn = c_old.Pn
				c.Pt = c_old.Pt
				c.Pnb = c_old.Pnb
			else:
				c.Pn = 0.0
				c.Pt = 0.0
				c.Pnb = 0.0
		else:
			merged_contacts[i] = new_contacts[i];

	for i in num_new_contacts:
		contacts[i] = merged_contacts[i];

	num_contacts = num_new_contacts;


func pre_step(inv_dt: float):
	var k_allowed_penetration = 0.01
	var k_bias_factor = 0.2 if PhysicsWorld.position_correction else 0.0;

	for i in num_contacts:
		var c = contacts[i];

		var r1 = c.position - body_1.position;
		var r2 = c.position - body_2.position;

		# Precompute normal mass, tangent mass, and bias.
		var rn1 = r1.dot(c.normal)
		var rn2 = r2.dot(c.normal)
		var k_normal = body_1.inv_mass + body_2.inv_mass
		k_normal += body_1.inv_I * (r1.dot(r1) - rn1 * rn1) + body_2.inv_I * (r2.dot(r2) - rn2 * rn2)
		c.mass_normal = 1.0 / k_normal

		var tangent = Math.cross_scalar(c.normal, 1.0)
		var rt1 = r1.dot(tangent)
		var rt2 = r2.dot(tangent)
		var k_tangent = body_1.inv_mass + body_2.inv_mass;
		k_tangent += body_1.inv_I * (r1.dot(r1) - rt1 * rt1) + body_2.inv_I * (r2.dot(r2) - rt2 * rt2)
		c.mass_tangent = 1.0 /  k_tangent

		c.bias = -k_bias_factor * inv_dt * min(0.0, c.separation + k_allowed_penetration)

		if PhysicsWorld.accumulate_impulses:
			# Apply normal + friction impulse
			var P = c.Pn * c.normal + c.Pt * tangent;

			body_1.velocity -= body_1.inv_mass * P;
			body_1.angular_velocity -= body_1.inv_I * r1.cross(P);

			body_2.velocity += body_2.inv_mass * P;
			body_2.angular_velocity += body_2.inv_I * r2.cross(P);


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
		var vn = dv.dot(c.normal);

		var d_Pn = c.mass_normal * (-vn + c.bias);

		if PhysicsWorld.accumulate_impulses:
			# Clamp the accumulated impulse.
			var Pn0 = c.Pn;
			c.Pn = max(Pn0 + d_Pn, 0.0);
			d_Pn = c.Pn - Pn0;
		else:
			d_Pn = max(d_Pn, 0.0);

		# Apply contact impulse.
		var Pn = d_Pn * c.normal;

		b1.velocity -= b1.inv_mass * Pn;
		b1.angular_velocity -= b1.inv_I * c.r1.cross(Pn)

		b2.velocity += b2.inv_mass * Pn;
		b2.angular_velocity += b2.inv_I * c.r2.cross(Pn)

		# Relative velocity at contact.
		dv = b2.velocity + Math.cross_vector(b2.angular_velocity, c.r2) - b1.velocity - Math.cross_vector(b1.angular_velocity, c.r1)

		var tangent = Math.cross_scalar(c.normal, 1.0)
		var vt = dv.dot(tangent)
		var d_Pt = c.mass_tangent * (-vt)

		if PhysicsWorld.accumulate_impulses:
			# Compute friction impulse.
			var max_Pt = friction * c.Pn;

			# Clamp friction.
			var old_tangent_impulse = c.Pt;
			c.Pt = clamp(old_tangent_impulse + d_Pt, -max_Pt, max_Pt);
			d_Pt = c.Pt - old_tangent_impulse;
		else:
			var max_Pt = friction * d_Pn;
			d_Pt = clamp(d_Pt, -max_Pt, max_Pt);

		# Apply contact impulse
		var Pt = d_Pt * tangent;

		b1.velocity -= b1.inv_mass * Pt;
		b1.angular_velocity -= b1.inv_I * c.r1.cross(Pt);

		b2.velocity += b2.inv_mass * Pt;
		b2.angular_velocity += b2.inv_I * c.r2.cross(Pt);

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
	EDGE1,
	EDGE2,
	EDGE3,
	EDGE4,
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
	var num_out = 0;

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
			v_out[num_out].fp = v_in[0].fp;
			v_out[num_out].fp.e.in_edge_1 = clip_edge;
			v_out[num_out].fp.e.in_edge_2 = EdgeNumbers.NO_EDGE;
		else:
			v_out[num_out].fp = v_in[1].fp;
			v_out[num_out].fp.e.out_edge_1 = clip_edge;
			v_out[num_out].fp.e.out_edge_2 = EdgeNumbers.NO_EDGE;

		num_out += 1

	return num_out


static func compute_incident_edge(c: Array, h: Vector2, pos: Vector2, Rot: Transform2D, normal: Vector2):
	# The normal is from the reference box. Convert it
	# to the incident boxe's frame and flip sign.
	var RotT = Rot.inverse();
	var n = -(RotT * normal);
	var n_abs = n.abs();

	if n_abs.x > n_abs.y:
		if (sign(n.x) > 0):
			c[0].v = Vector2(h.x, -h.y)
			c[0].fp.e.in_edge_2 = EdgeNumbers.EDGE3;
			c[0].fp.e.out_edge_2 = EdgeNumbers.EDGE4;

			c[1].v = Vector2(h.x, h.y)
			c[1].fp.e.in_edge_2 = EdgeNumbers.EDGE4;
			c[1].fp.e.out_edge_2 = EdgeNumbers.EDGE1;
		else:
			c[0].v = Vector2(-h.x, h.y)
			c[0].fp.e.in_edge_2 = EdgeNumbers.EDGE1;
			c[0].fp.e.out_edge_2 = EdgeNumbers.EDGE2;

			c[1].v = Vector2(-h.x, -h.y)
			c[1].fp.e.in_edge_2 = EdgeNumbers.EDGE2;
			c[1].fp.e.out_edge_2 = EdgeNumbers.EDGE3;
	else:
		if (sign(n.y) > 0):
			c[0].v = Vector2(h.x, h.y)
			c[0].fp.e.in_edge_2 = EdgeNumbers.EDGE4;
			c[0].fp.e.out_edge_2 = EdgeNumbers.EDGE1;

			c[1].v = Vector2(-h.x, h.y)
			c[1].fp.e.in_edge_2 = EdgeNumbers.EDGE1;
			c[1].fp.e.out_edge_2 = EdgeNumbers.EDGE2;
		else:
			c[0].v = Vector2(-h.x, -h.y)
			c[0].fp.e.in_edge_2 = EdgeNumbers.EDGE2;
			c[0].fp.e.out_edge_2 = EdgeNumbers.EDGE3;

			c[1].v = Vector2(h.x, -h.y)
			c[1].fp.e.in_edge_2 = EdgeNumbers.EDGE3;
			c[1].fp.e.out_edge_2 = EdgeNumbers.EDGE4;

	c[0].v = pos + Rot * c[0].v;
	c[1].v = pos + Rot * c[1].v;


# The normal points from A to B.
static func collide(contacts, var body_a, var body_b):
	# Setup.
	var hA = 0.5 * body_a.width;
	var hB = 0.5 * body_b.width;

	var posA = body_a.position;
	var posB = body_b.position;

	var RotA = Transform2D(body_a.rotation, Vector2())
	var RotB = Transform2D(body_b.rotation, Vector2())

	var RotAT = RotA.inverse()
	var RotBT = RotB.inverse()

	var dp = posB - posA;
	var dA = RotAT * dp;
	var dB = RotBT * dp;

	var C = RotAT * RotB;
	var absC = Transform2D()
	absC.x = C.x.abs()
	absC.y = C.y.abs()
	var absCT = absC.inverse()

	# Box A faces.
	var faceA = dA.abs() - hA - absC * hB
	if faceA.x > 0.0 or faceA.y > 0.0:
		return 0;

	# Box B faces.
	var faceB = dB.abs() - absCT * hA - hB
	if faceB.x > 0.0 or faceB.y > 0.0:
		return 0;

	# Box A faces.
	var axis = Axis.FACE_A_X
	var separation = faceA.x
	var normal = RotA.x if dA.x > 0.0 else -RotA.x

	var relative_tol = 0.95
	var absolute_tol = 0.01

	if faceA.y > relative_tol * separation + absolute_tol * hA.y:
		axis = Axis.FACE_A_Y;
		separation = faceA.y;
		normal = RotA.y if dA.y > 0.0 else -RotA.y;

	# Box B faces.
	if faceB.x > relative_tol * separation + absolute_tol * hB.x:
		axis = Axis.FACE_B_X;
		separation = faceB.x;
		normal = RotB.x if dB.x > 0.0 else -RotB.x;

	if faceB.y > relative_tol * separation + absolute_tol * hB.y:
		axis = Axis.FACE_B_Y;
		separation = faceB.y;
		normal = RotB.y if dB.y > 0.0 else -RotB.y;

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
			front_normal = normal;
			front = posA.dot(front_normal) + hA.x;
			side_normal = RotA.y;
			var side = posA.dot(side_normal);
			neg_side = -side + hA.y;
			pos_side =  side + hA.y;
			neg_edge = EdgeNumbers.EDGE3;
			pos_edge = EdgeNumbers.EDGE1;
			compute_incident_edge(incident_edge, hB, posB, RotB, front_normal);
		Axis.FACE_A_Y:
			front_normal = normal;
			front = posA.dot(front_normal) + hA.y;
			side_normal = RotA.x;
			var side = posA.dot(side_normal);
			neg_side = -side + hA.x;
			pos_side =  side + hA.x;
			neg_edge = EdgeNumbers.EDGE2;
			pos_edge = EdgeNumbers.EDGE4;
			compute_incident_edge(incident_edge, hB, posB, RotB, front_normal);
		Axis.FACE_B_X:
			front_normal = -normal;
			front = posB.dot(front_normal) + hB.x;
			side_normal = RotB.y;
			var side = posB.dot(side_normal);
			neg_side = -side + hB.y;
			pos_side =  side + hB.y;
			neg_edge = EdgeNumbers.EDGE3;
			pos_edge = EdgeNumbers.EDGE1;
			compute_incident_edge(incident_edge, hA, posA, RotA, front_normal);
		Axis.FACE_B_Y:
			front_normal = -normal;
			front = posB.dot(front_normal) + hB.y;
			side_normal = RotB.x;
			var side = posB.dot(side_normal);
			neg_side = -side + hB.x;
			pos_side =  side + hB.x;
			neg_edge = EdgeNumbers.EDGE2;
			pos_edge = EdgeNumbers.EDGE4;
			compute_incident_edge(incident_edge, hA, posA, RotA, front_normal);

	# Clip other face with 5 box planes (1 face plane, 4 edge planes).
	var clip_points_1 = []
	clip_points_1.push_back(ClipVertex.new())
	clip_points_1.push_back(ClipVertex.new())
	var clip_points_2 = []
	clip_points_2.push_back(ClipVertex.new())
	clip_points_2.push_back(ClipVertex.new())

	# Clip to box side 1.
	var np = clip_segment_to_line(clip_points_1, incident_edge, -side_normal, neg_side, neg_edge);
	if np < 2:
		return 0

	# Clip to negative box side 1.
	np = clip_segment_to_line(clip_points_2, clip_points_1,  side_normal, pos_side, pos_edge);
	if np < 2:
		return 0

	# Now clip_points_2 contains the clipping points..
	# Due to roundoff, it is possible that clipping removes all points..
	var num_contacts = 0;
	for i in 2:
		var sep = front_normal.dot(clip_points_2[i].v) - front;
		if sep <= 0:
			contacts[num_contacts].separation = sep;
			contacts[num_contacts].normal = normal;
			# Slide contact point onto reference face (easy to cull).
			contacts[num_contacts].position = clip_points_2[i].v - sep * front_normal;
			contacts[num_contacts].feature = clip_points_2[i].fp;
			if axis == Axis.FACE_B_X or axis == Axis.FACE_B_Y:
				flip(contacts[num_contacts].feature);
			num_contacts += 1

	return num_contacts;
