# python-fcl
### Python Interface for the Flexible Collision Library

Python-FCL is an (unofficial) Python interface for the [Flexible Collision Library (FCL)](https://github.com/flexible-collision-library/fcl),
an excellent C++ library for performing proximity and collision queries on pairs of geometric models.
Currently, this package is targeted for FCL 0.7.0.

This package supports three types of proximity queries for pairs of geometric models:
* __Collision Detection__: Detecting whether two models overlap (and optionally where).
* __Distance Computation__: Computing the minimum distance between a pair of models.
* __Continuous Collision Detection__: Detecting whether two models overlap during motion (and optionally the time of contact).

This package also supports most of FCL's object shapes, including:
* TriangleP
* Box
* Sphere
* Ellipsoid
* Capsule
* Cone
* Convex
* Cylinder
* Half-Space
* Plane
* Mesh
* OcTree

## Installation

First, install [octomap](https://github.com/OctoMap/octomap), which is necessary to use OcTree. For Ubuntu, use `sudo apt-get install liboctomap-dev`.
Second, install FCL using the instructions provided [here](https://github.com/flexible-collision-library/fcl).
If you're on Ubuntu 17.04 or newer, you can install FCL using `sudo apt-get install libfcl-dev`.
Otherwise, just compile FCL from source -- it's quick and easy, and its dependencies are all easily installed via `apt` or `brew`.
Note: the provided install scripts (under `build_dependencies`) can automate this process as well.

In order to install the Python wrappers for FCL, simply run
```shell
pip install python-fcl
```

## Objects

### Collision Objects
The primary construct in FCL is the `CollisionObject`, which forms the backbone of all collision and distance computations.
A `CollisionObject` consists of two components -- its geometry, defined by a `CollisionGeometry` object, and its pose, defined by a `Transform` object.

#### Collision Geometries
There are two main types of `CollisionGeometry` objects -- geometric primitives, such as boxes and spheres,
and arbitrary triangular meshes.
Here's some examples of how to instantiate geometric primitives.
Note that the box, sphere, ellipsoid, capsule, cone, and cylinder are all centered at the origin.

```python
import numpy as np
import fcl

v1 = np.array([1.0, 2.0, 3.0])
v2 = np.array([2.0, 1.0, 3.0])
v3 = np.array([3.0, 2.0, 1.0])
x, y, z = 1, 2, 3
rad, lz = 1.0, 3.0
n = np.array([1.0, 0.0, 0.0])
d = 5.0

t = fcl.TriangleP(v1, v2, v3) # Triangle defined by three points
b = fcl.Box(x, y, z)          # Axis-aligned box with given side lengths
s = fcl.Sphere(rad)           # Sphere with given radius
e = fcl.Ellipsoid(x, y, z)    # Axis-aligned ellipsoid with given radii
c = fcl.Capsule(rad, lz)      # Capsule with given radius and height along z-axis
c = fcl.Cone(rad, lz)         # Cone with given radius and cylinder height along z-axis
c = fcl.Cylinder(rad, lz)     # Cylinder with given radius and height along z-axis
h = fcl.Halfspace(n, d)       # Half-space defined by {x : <n, x> < d}
p = fcl.Plane(n, d)           # Plane defined by {x : <n, x> = d}
```

Triangular meshes are wrapped by the `BVHModel` class, and they are instantiated a bit differently.
```python
verts = np.array([[1.0, 1.0, 1.0],
                  [2.0, 1.0, 1.0],
                  [1.0, 2.0, 1.0],
                  [1.0, 1.0, 2.0]])
tris  = np.array([[0,2,1],
                  [0,3,2],
                  [0,1,3],
                  [1,2,3]])

m = fcl.BVHModel()
m.beginModel(len(verts), len(tris))
m.addSubModel(verts, tris)
m.endModel()
```

If the mesh is convex, such as the example above, you can also wrap it in the `Convex` class. Note that the instantiation is a bit different because the `Convex` class supports arbitrary polygons for each face of the convex object.
```python
verts = np.array([[1.0, 1.0, 1.0],
                  [2.0, 1.0, 1.0],
                  [1.0, 2.0, 1.0],
                  [1.0, 1.0, 2.0]])
tris  = np.array([[0,2,1],
                  [0,3,2],
                  [0,1,3],
                  [1,2,3]])
faces = np.concatenate((3 * np.ones((len(tris), 1), dtype=np.int64), tris), axis=1).flatten()
c = fcl.Convex(verts, len(tris), faces)
```

#### Transforms
In addition to a `CollisionGeometry`, a `CollisionObject` requires a `Transform`, which tells FCL where the `CollisionGeometry` is actually located in the world.
All `Transform` objects specify a rigid transformation (i.e. a rotation and a translation).
The translation is always a 3-entry vector, while the rotation can be specified by a 3x3 rotation matrix or a 4-entry quaternion.

Here are some examples of possible ways to instantiate and manipulate a `Transform`.

```python
R = np.array([[0.0, -1.0, 0.0],
              [1.0,  0.0, 0.0],
              [0.0,  0.0, 1.0]])
T = np.array([1.0, 2.0, 3.0])
q = np.array([0.707, 0.0, 0.0, 0.707])

tf = fcl.Transform()     # Default gives identity transform
tf = fcl.Transform(q)    # Quaternion rotation, zero translation
tf = fcl.Transform(R)    # Matrix rotation, zero translation
tf = fcl.Transform(T)    # Translation, identity rotation
tf = fcl.Transform(q, T) # Quaternion rotation and translation
tf = fcl.Transform(R, T) # Matrix rotation and translation
tf1 = fcl.Transform(tf)  # Can also initialize with another Transform
```

Now, given a `CollisionGeometry` and a `Transform`, we can create a `CollisionObject`:

```python
t = fcl.Transform(R, T)
b = fcl.Box(x, y, z)
obj = fcl.CollisionObject(b, t)
```

The transform of a collision object can be modified in-place:
```python
t1 = fcl.Transform(R1, T1)
obj.setTransform(t1)   # Using a transform
obj.setRotation(R2)    # Specifying components individually
obj.setTranslation(T2)
obj.setQuatRotation(q2)
```
## Commands

### Pairwise Operations

Given a pair of collision objects, this library supports three types of queries:
* __Collision Detection__
* __Distance Computation__
* __Continuous Collision Detection__

The interfaces for each of these operations follow a common pipeline.
First, a query request data structure is initialized and populated with parameters.
Then, an empty query response structure is initialized.
Finally, the query function is called with the two `CollisionObject` items, the request structure, and the response structure as arguments.
The query function returns a scalar result, and any additional information is stored in the query result data structure.
Examples of all three operations are shown below.

#### Collision Checking

```python
g1 = fcl.Box(1,2,3)
t1 = fcl.Transform()
o1 = fcl.CollisionObject(g1, t1)

g2 = fcl.Cone(1,3)
t2 = fcl.Transform()
o2 = fcl.CollisionObject(g2, t2)

request = fcl.CollisionRequest()
result = fcl.CollisionResult()

ret = fcl.collide(o1, o2, request, result)
```

After calling `fcl.collide()`, `ret` contains the number of contacts generated between the two objects,
and `result` contains information about the collision and contacts.
For more information about available parameters for collision requests and results,
see `fcl/collision_data.py`.

#### Distance Checking

```python
g1 = fcl.Box(1,2,3)
t1 = fcl.Transform()
o1 = fcl.CollisionObject(g1, t1)

g2 = fcl.Cone(1,3)
t2 = fcl.Transform()
o2 = fcl.CollisionObject(g2, t2)

request = fcl.DistanceRequest()
result = fcl.DistanceResult()

ret = fcl.distance(o1, o2, request, result)
```

After calling `fcl.distance()`, `ret` contains the minimum distance between the two objects
and `result` contains information about the closest points on the objects.
If `ret` is negative, the objects are in collision.
For more information about available parameters for distance requests and results,
see `fcl/collision_data.py`.

#### Continuous Collision Checking

```python
g1 = fcl.Box(1,2,3)
t1 = fcl.Transform()
o1 = fcl.CollisionObject(g1, t1)
t1_final = fcl.Transform(np.array([1.0, 0.0, 0.0]))

g2 = fcl.Cone(1,3)
t2 = fcl.Transform()
o2 = fcl.CollisionObject(g2, t2)
t2_final = fcl.Transform(np.array([-1.0, 0.0, 0.0]))

request = fcl.ContinuousCollisionRequest()
result = fcl.ContinuousCollisionResult()

ret = fcl.continuousCollide(o1, t1_final, o2, t2_final, request, result)
```

After calling `fcl.continuousCollide()`, `ret` contains the time of contact in (0,1), or 1.0 if the objects did not collide during movement from their initial poses to their final poses.
Additionally, `result` contains information about the collision time and status.
For more information about available parameters for continuous collision requests and results,
see `fcl/collision_data.py`.

### Broadphase Checking
In addition to pairwise checks, FCL supports broadphase collision/distance queries between groups of objects and can avoid n-squared complexity.
Specifically, `CollisionObject` items are registered with a `DynamicAABBTreeCollisionManager` before collision or distance checking is performed.

Three types of checks are possible:
* One-to-many: Collision/distance checking between a stand-alone `CollisionObject` and all objects managed by a manager.
* Internal many-to-many: Pairwise collision/distance checking between all pairs of objects managed by a manager.
* Group many-to-many: Pairwise collision/distance checking between items from two managers.

In general, the collision methods can return all contact pairs, while the distance methods will just return the single closest distance between any pair of objects.
Here are some examples of managed collision checking.
The methods take a callback function -- use the defaults from `python-fcl` unless you have a special use case -- and a wrapper object, either `CollisionData` or `DistanceData`, that wraps a request-response pair. This object also has a field, `done`, that tells the recursive collision checker when to quit.
Be sure to use a new `Data` object for each request or set the `done` attribute to `False` before reusing one.

```python
objs1 = [fcl.CollisionObject(box), fcl.CollisionObject(sphere)]
objs2 = [fcl.CollisionObject(cone), fcl.CollisionObject(mesh)]

manager1 = fcl.DynamicAABBTreeCollisionManager()
manager2 = fcl.DynamicAABBTreeCollisionManager()

manager1.registerObjects(objs1)
manager2.registerObjects(objs2)

manager1.setup()
manager2.setup()

#=====================================================================
# Managed internal (sub-n^2) collision checking
#=====================================================================
cdata = fcl.CollisionData()
manager1.collide(cdata, fcl.defaultCollisionCallback)
print 'Collision within manager 1?: {}'.format(cdata.result.is_collision)

##=====================================================================
## Managed internal (sub-n^2) distance checking
##=====================================================================
ddata = fcl.DistanceData()
manager1.distance(ddata, fcl.defaultDistanceCallback)
print 'Closest distance within manager 1?: {}'.format(ddata.result.min_distance)

#=====================================================================
# Managed one to many collision checking
#=====================================================================
req = fcl.CollisionRequest(num_max_contacts=100, enable_contact=True)
rdata = fcl.CollisionData(request = req)

manager1.collide(fcl.CollisionObject(mesh), rdata, fcl.defaultCollisionCallback)
print 'Collision between manager 1 and Mesh?: {}'.format(rdata.result.is_collision)
print 'Contacts:'
for c in rdata.result.contacts:
    print '\tO1: {}, O2: {}'.format(c.o1, c.o2)

#=====================================================================
# Managed many to many collision checking
#=====================================================================
rdata = fcl.CollisionData(request = req)
manager1.collide(manager2, rdata, fcl.defaultCollisionCallback)
print 'Collision between manager 1 and manager 2?: {}'.format(rdata.result.is_collision)
print 'Contacts:'
for c in rdata.result.contacts:
    print '\tO1: {}, O2: {}'.format(c.o1, c.o2)
```

### Extracting Which Objects Are In Collision

To determine which objects are actually in collision, you'll need parse the collision data's contacts and use an additional external data structure.

Specifically, the `fcl.CollisionData` object that is passed into any `collide()` call has an internal set of contacts, stored in `cdata.result.contacts`.
This object is a simple list of `Contact` objects, each of which represents a contact point between two objects.
Each contact object has two attributes, `o1` and `o2`, that store references to the original `fcl.CollisionGeometry` objects were created for the two `fcl.CollisionObject` objects that are in collision.
This is a bit wonky, but it's part of the FCL API.

Therefore, all you have to do is make a map from the `id` of each `fcl.CollisionGeometry` object to either the actual `fcl.CollisionObject` it corresponds to or to some string identifier for each object.
Then, you can iterate over `cdata.result.contacts`, extract `o1` and `o2`, apply the built-in `id()` function to each, and find the corresponding data you want in your map.

Here's an example.

```python
import fcl
import numpy as np

# Create collision geometry and objects
geom1 = fcl.Cylinder(1.0, 1.0)
obj1 = fcl.CollisionObject(geom1)

geom2 = fcl.Cylinder(1.0, 1.0)
obj2 = fcl.CollisionObject(geom2, fcl.Transform(np.array([0.0, 0.0, 0.3])))

geom3 = fcl.Cylinder(1.0, 1.0)
obj3 = fcl.CollisionObject(geom3, fcl.Transform(np.array([0.0, 0.0, 3.0])))

geoms = [geom1, geom2, geom3]
objs = [obj1, obj2, obj3]
names = ['obj1', 'obj2', 'obj3']

# Create map from geometry IDs to objects
geom_id_to_obj = { id(geom) : obj for geom, obj in zip(geoms, objs) }

# Create map from geometry IDs to string names
geom_id_to_name = { id(geom) : name for geom, name in zip(geoms, names) }

# Create manager
manager = fcl.DynamicAABBTreeCollisionManager()
manager.registerObjects(objs)
manager.setup()

# Create collision request structure
crequest = fcl.CollisionRequest(num_max_contacts=100, enable_contact=True)
cdata = fcl.CollisionData(crequest, fcl.CollisionResult())

# Run collision request
manager.collide(cdata, fcl.defaultCollisionCallback)

# Extract collision data from contacts and use that to infer set of
# objects that are in collision
objs_in_collision = set()

for contact in cdata.result.contacts:
    # Extract collision geometries that are in contact
    coll_geom_0 = contact.o1
    coll_geom_1 = contact.o2

    # Get their names
    coll_names = [geom_id_to_name[id(coll_geom_0)], geom_id_to_name[id(coll_geom_1)]]
    coll_names = tuple(sorted(coll_names))
    objs_in_collision.add(coll_names)

for coll_pair in objs_in_collision:
    print('Object {} in collision with object {}!'.format(coll_pair[0], coll_pair[1]))
```

```
>>> Object obj1 in collision with object obj2!
```
For more examples, see `examples/example.py`.
