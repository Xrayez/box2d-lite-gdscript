class_name Collide

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
	var fp = Box2DArbiter.FeaturePair.new()


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
	clip_points_1.resize(2)
	var clip_points_2 = []
	clip_points_2.resize(2)

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
