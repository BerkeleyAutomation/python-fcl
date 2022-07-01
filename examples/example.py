import numpy as np

import fcl


def print_collision_result(o1_name, o2_name, result):
    print("Collision between {} and {}:".format(o1_name, o2_name))
    print("-" * 30)
    print("Collision?: {}".format(result.is_collision))
    print("Number of contacts: {}".format(len(result.contacts)))
    print("")


def print_continuous_collision_result(o1_name, o2_name, result):
    print("Continuous collision between {} and {}:".format(o1_name, o2_name))
    print("-" * 30)
    print("Collision?: {}".format(result.is_collide))
    print("Time of collision: {}".format(result.time_of_contact))
    print("")


def print_distance_result(o1_name, o2_name, result):
    print("Distance between {} and {}:".format(o1_name, o2_name))
    print("-" * 30)
    print("Distance: {}".format(result.min_distance))
    print("Closest Points:")
    print(result.nearest_points[0])
    print(result.nearest_points[1])
    print("")


# Create simple geometries
box = fcl.Box(1.0, 2.0, 3.0)
sphere = fcl.Sphere(4.0)
cone = fcl.Cone(5.0, 6.0)
cyl = fcl.Cylinder(2.0, 2.0)

verts = np.array([[1.0, 1.0, 1.0], [2.0, 1.0, 1.0], [1.0, 2.0, 1.0], [1.0, 1.0, 2.0]])
tris = np.array([[0, 2, 1], [0, 3, 2], [0, 1, 3], [1, 2, 3]])

# Create mesh geometry
mesh = fcl.BVHModel()
mesh.beginModel(len(verts), len(tris))
mesh.addSubModel(verts, tris)
mesh.endModel()

# =====================================================================
# Pairwise collision checking
# =====================================================================
print("=" * 60)
print("Testing Pairwise Collision Checking")
print("=" * 60)
print("")

req = fcl.CollisionRequest(enable_contact=True)
res = fcl.CollisionResult()

n_contacts = fcl.collide(
    fcl.CollisionObject(box, fcl.Transform()),
    fcl.CollisionObject(cone, fcl.Transform()),
    req,
    res,
)
print_collision_result("Box", "Cone", res)

n_contacts = fcl.collide(
    fcl.CollisionObject(box, fcl.Transform()),
    fcl.CollisionObject(cyl, fcl.Transform(np.array([6.0, 0.0, 0.0]))),
    req,
    res,
)
print_collision_result("Box", "Cylinder", res)

n_contacts = fcl.collide(
    fcl.CollisionObject(mesh, fcl.Transform(np.array([0.0, 0.0, -1.0]))),
    fcl.CollisionObject(cyl, fcl.Transform()),
    req,
    res,
)
print_collision_result("Box", "Mesh", res)

# =====================================================================
# Pairwise distance checking
# =====================================================================
print("=" * 60)
print("Testing Pairwise Distance Checking")
print("=" * 60)
print("")

req = fcl.DistanceRequest(enable_nearest_points=True, enable_signed_distance=True)
res = fcl.DistanceResult()

dist = fcl.distance(
    fcl.CollisionObject(box, fcl.Transform()),
    fcl.CollisionObject(cone, fcl.Transform()),
    req,
    res,
)
print_distance_result("Box", "Cone", res)

dist = fcl.distance(
    fcl.CollisionObject(box, fcl.Transform()),
    fcl.CollisionObject(cyl, fcl.Transform(np.array([6.0, 0.0, 0.0]))),
    req,
    res,
)
print_distance_result("Box", "Cylinder", res)

dist = fcl.distance(
    fcl.CollisionObject(box, fcl.Transform()),
    fcl.CollisionObject(box, fcl.Transform(np.array([1.01, 0.0, 0.0]))),
    req,
    res,
)
print_distance_result("Box", "Box", res)

# =====================================================================
# Pairwise continuous collision checking
# =====================================================================
print("=" * 60)
print("Testing Pairwise Continuous Collision Checking")
print("=" * 60)
print("")

req = fcl.ContinuousCollisionRequest()
res = fcl.ContinuousCollisionResult()

dist = fcl.continuousCollide(
    fcl.CollisionObject(box, fcl.Transform()),
    fcl.Transform(np.array([5.0, 0.0, 0.0])),
    fcl.CollisionObject(cyl, fcl.Transform(np.array([5.0, 0.0, 0.0]))),
    fcl.Transform(np.array([0.0, 0.0, 0.0])),
    req,
    res,
)
print_continuous_collision_result("Box", "Cylinder", res)

# =====================================================================
# Managed collision checking
# =====================================================================
print("=" * 60)
print("Testing Managed Collision and Distance Checking")
print("=" * 60)
print("")
objs1 = [
    fcl.CollisionObject(box, fcl.Transform(np.array([20, 0, 0]))),
    fcl.CollisionObject(sphere),
]
objs2 = [fcl.CollisionObject(cone), fcl.CollisionObject(mesh)]
objs3 = [fcl.CollisionObject(box), fcl.CollisionObject(sphere)]

manager1 = fcl.DynamicAABBTreeCollisionManager()
manager2 = fcl.DynamicAABBTreeCollisionManager()
manager3 = fcl.DynamicAABBTreeCollisionManager()

manager1.registerObjects(objs1)
manager2.registerObjects(objs2)
manager3.registerObjects(objs3)

manager1.setup()
manager2.setup()
manager3.setup()

# =====================================================================
# Managed internal (n^2) collision checking
# =====================================================================
cdata = fcl.CollisionData()
manager1.collide(cdata, fcl.defaultCollisionCallback)
print("Collision within manager 1?: {}".format(cdata.result.is_collision))
print("")

cdata = fcl.CollisionData()
manager2.collide(cdata, fcl.defaultCollisionCallback)
print("Collision within manager 2?: {}".format(cdata.result.is_collision))
print("")

# =====================================================================
# Managed internal (n^2) distance checking
# =====================================================================
ddata = fcl.DistanceData()
manager1.distance(ddata, fcl.defaultDistanceCallback)
print("Closest distance within manager 1?: {}".format(ddata.result.min_distance))
print("")

ddata = fcl.DistanceData()
manager2.distance(ddata, fcl.defaultDistanceCallback)
print("Closest distance within manager 2?: {}".format(ddata.result.min_distance))
print("")

# =====================================================================
# Managed one to many collision checking
# =====================================================================
req = fcl.CollisionRequest(num_max_contacts=100, enable_contact=True)
rdata = fcl.CollisionData(request=req)

manager1.collide(fcl.CollisionObject(mesh), rdata, fcl.defaultCollisionCallback)
print("Collision between manager 1 and Mesh?: {}".format(rdata.result.is_collision))
print("Contacts:")
for c in rdata.result.contacts:
    print("\tO1: {}, O2: {}".format(c.o1, c.o2))
print("")

# =====================================================================
# Managed many to many collision checking
# =====================================================================
rdata = fcl.CollisionData(request=req)
manager3.collide(manager2, rdata, fcl.defaultCollisionCallback)
print("Collision between manager 2 and manager 3?: {}".format(rdata.result.is_collision))
print("Contacts:")
for c in rdata.result.contacts:
    print("\tO1: {}, O2: {}".format(c.o1, c.o2))
print("")
