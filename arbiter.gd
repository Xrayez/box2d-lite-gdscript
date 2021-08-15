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
	if b1.get_instance_id() < b2.get_instance_id():
		body_1 = b1
		body_2 = b2
	else:
		body_1 = b2
		body_2 = b1

	num_contacts = Collide.collide(contacts, body_1, body_2)

	friction = sqrt(body_1.friction * body_2.friction)


func update(new_contacts: Array, num_new_contacts: int):
	var merged_contacts = []
	merged_contacts.resize(2)

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

		var tangent = c.normal.cross(1.0);
		var rt1 = r1.dot(tangent)
		var rt2 = r2.dot(tangent)
		var k_tangent = body_1.inv_mass + body_2.inv_mass;
		k_tangent += body_1.inv_I * (r1.dot(r1) - rt1 * rt1) + body_2.inv_I * (r2.dot(r2) - rt2 * rt2)
		c.massTangent = 1.0 /  k_tangent

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
		var dv = b2.velocity + b2.angular_velocity.cross(c.r2) - b1.velocity - b1.angular_velocity.cross(c.r1)

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
		b1.angular_velocity -= b1.inv_I * c.r1.cross(Pn);

		b2.velocity += b2.inv_mass * Pn;
		b2.angular_velocity += b2.inv_I * c.r2.cross(Pn);

		# Relative velocity at contact.
		dv = b2.velocity + b2.angular_velocity.cross(c.r2) - b1.velocity - b1.angular_velocity.cross(c.r1)

		var tangent = c.normal.cross(1.0);
		var vt = dv.dot(tangent);
		var d_Pt = c.massTangent * (-vt);

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
