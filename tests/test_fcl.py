import unittest

import numpy as np

import fcl


class TestFCL(unittest.TestCase):
    def setUp(self):
        verts = np.array(
            [[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
        )
        tris = np.array([[0, 2, 1], [0, 3, 2], [0, 1, 3], [1, 2, 3]])
        mesh = fcl.BVHModel()
        mesh.beginModel(len(verts), len(tris))
        mesh.addSubModel(verts, tris)
        mesh.endModel()

        verts = np.array(
            [
                [-0.5, -0.5, -0.5],
                [-0.5, 0.5, -0.5],
                [-0.5, 0.5, 0.5],
                [-0.5, -0.5, 0.5],
                [0.5, -0.5, -0.5],
                [0.5, 0.5, -0.5],
                [0.5, 0.5, 0.5],
                [0.5, -0.5, 0.5],
            ]
        )
        quads = np.array(
            [
                [0, 3, 2, 1],
                [4, 5, 6, 7],
                [4, 7, 3, 0],
                [5, 1, 2, 6],
                [4, 0, 1, 5],
                [7, 6, 2, 3],
            ]
        )

        faces = np.concatenate(
            (np.ones((len(quads), 1), dtype=np.int32) * 4, quads), axis=1
        ).flatten()
        convex = fcl.Convex(verts, len(quads), faces)

        self.geometry = {
            "box": fcl.Box(1.0, 1.0, 1.0),
            "sphere": fcl.Sphere(1.0),
            "cone": fcl.Cone(1.0, 1.0),
            "cylinder": fcl.Cylinder(1.0, 1.0),
            "mesh": mesh,
            "convex": convex,
        }

        self.crequest = fcl.CollisionRequest(num_max_contacts=100, enable_contact=True)
        self.drequest = fcl.DistanceRequest(
            enable_nearest_points=True, enable_signed_distance=True
        )

        self.x_axis_rot = np.array([[1.0, 0.0, 0.0], [0.0, 0.0, -1.0], [0.0, 1.0, 0.0]])

    def test_pairwise_collisions(self):
        result = fcl.CollisionResult()

        box = fcl.CollisionObject(self.geometry["box"])
        cone = fcl.CollisionObject(self.geometry["cone"])
        mesh = fcl.CollisionObject(self.geometry["mesh"])
        convex = fcl.CollisionObject(self.geometry["convex"])

        result = fcl.CollisionResult()
        ret = fcl.collide(box, cone, self.crequest, result)
        self.assertTrue(ret > 0)
        self.assertTrue(result.is_collision)

        result = fcl.CollisionResult()
        ret = fcl.collide(convex, cone, self.crequest, result)
        self.assertTrue(ret > 0)
        self.assertTrue(result.is_collision)

        result = fcl.CollisionResult()
        ret = fcl.collide(box, mesh, self.crequest, result)
        self.assertTrue(ret > 0)
        self.assertTrue(result.is_collision)

        result = fcl.CollisionResult()
        ret = fcl.collide(convex, mesh, self.crequest, result)
        self.assertTrue(ret > 0)
        self.assertTrue(result.is_collision)

        result = fcl.CollisionResult()
        ret = fcl.collide(cone, mesh, self.crequest, result)
        self.assertTrue(ret > 0)
        self.assertTrue(result.is_collision)

        cone.setTranslation(np.array([0.0, 0.0, -0.6]))

        result = fcl.CollisionResult()
        ret = fcl.collide(box, cone, self.crequest, result)
        self.assertTrue(ret > 0)
        self.assertTrue(result.is_collision)

        result = fcl.CollisionResult()
        ret = fcl.collide(convex, cone, self.crequest, result)
        self.assertTrue(ret > 0)
        self.assertTrue(result.is_collision)

        result = fcl.CollisionResult()
        ret = fcl.collide(cone, mesh, self.crequest, result)
        self.assertTrue(ret == 0)
        self.assertFalse(result.is_collision)

        cone.setTranslation(np.array([0.0, -0.9, 0.0]))
        cone.setRotation(self.x_axis_rot)

        result = fcl.CollisionResult()
        ret = fcl.collide(box, cone, self.crequest, result)
        self.assertTrue(ret > 0)
        self.assertTrue(result.is_collision)

        result = fcl.CollisionResult()
        ret = fcl.collide(convex, cone, self.crequest, result)
        self.assertTrue(ret > 0)
        self.assertTrue(result.is_collision)

        cone.setTranslation(np.array([0.0, -1.1, 0.0]))

        result = fcl.CollisionResult()
        ret = fcl.collide(box, cone, self.crequest, result)
        self.assertTrue(ret == 0)
        self.assertFalse(result.is_collision)

        result = fcl.CollisionResult()
        ret = fcl.collide(convex, cone, self.crequest, result)
        self.assertTrue(ret == 0)
        self.assertFalse(result.is_collision)

    def test_pairwise_distances(self):
        result = fcl.DistanceResult()

        box = fcl.CollisionObject(self.geometry["box"])
        cone = fcl.CollisionObject(self.geometry["cone"])
        mesh = fcl.CollisionObject(self.geometry["mesh"])
        convex = fcl.CollisionObject(self.geometry["convex"])

        result = fcl.DistanceResult()
        ret_box = fcl.distance(box, cone, self.drequest, result)
        self.assertTrue(ret_box < 0)

        result = fcl.DistanceResult()
        ret_convex = fcl.distance(convex, cone, self.drequest, result)
        self.assertTrue(ret_convex < 0)
        self.assertAlmostEqual(ret_convex, ret_box, places=6)

        result = fcl.DistanceResult()
        ret_box = fcl.distance(box, mesh, self.drequest, result)
        self.assertTrue(ret_box < 0)

        result = fcl.DistanceResult()
        ret_convex = fcl.distance(convex, mesh, self.drequest, result)
        self.assertTrue(ret_convex < 0)
        self.assertAlmostEqual(ret_convex, ret_box, places=6)

        result = fcl.DistanceResult()
        ret = fcl.distance(cone, mesh, self.drequest, result)
        self.assertTrue(ret < 0)

        cone.setTranslation(np.array([0.0, 0.0, -0.6]))

        result = fcl.DistanceResult()
        ret = fcl.distance(box, cone, self.drequest, result)
        self.assertTrue(ret < 0)

        result = fcl.DistanceResult()
        ret = fcl.distance(cone, mesh, self.drequest, result)
        self.assertAlmostEqual(ret, 0.1, places=6)

        cone.setTranslation(np.array([0.0, -0.9, 0.0]))
        cone.setRotation(self.x_axis_rot)

        result = fcl.DistanceResult()
        ret_box = fcl.distance(box, cone, self.drequest, result)
        self.assertTrue(ret_box < 0)

        result = fcl.DistanceResult()
        ret_convex = fcl.distance(convex, cone, self.drequest, result)
        self.assertTrue(ret_convex < 0)
        self.assertAlmostEqual(ret_box, ret_convex, places=6)

        cone.setTranslation(np.array([0.0, -1.1, 0.0]))

        result = fcl.DistanceResult()
        ret = fcl.distance(box, cone, self.drequest, result)
        self.assertAlmostEqual(ret, 0.1, places=6)

        result = fcl.DistanceResult()
        ret = fcl.distance(convex, cone, self.drequest, result)
        self.assertAlmostEqual(ret, 0.1, places=6)

    def test_pairwise_continuous_collisions(self):
        request = fcl.ContinuousCollisionRequest()
        result = fcl.ContinuousCollisionResult()

        box = fcl.CollisionObject(self.geometry["box"])
        cone = fcl.CollisionObject(
            self.geometry["cone"], fcl.Transform(np.array([0.0, 0.0, -2.0]))
        )

        ret = fcl.continuousCollide(
            box, fcl.Transform(), cone, fcl.Transform(), request, result
        )

        """
        ## WHY DOES THIS FAIL ##
        self.assertTrue(result.is_collide)
        self.assertAlmostEqual(0.625, ret)
        """

    def test_managed_collisions(self):
        manager1 = fcl.DynamicAABBTreeCollisionManager()
        manager2 = fcl.DynamicAABBTreeCollisionManager()
        manager3 = fcl.DynamicAABBTreeCollisionManager()

        objs1 = [
            fcl.CollisionObject(self.geometry["box"]),
            fcl.CollisionObject(self.geometry["cylinder"]),
        ]
        objs2 = [
            fcl.CollisionObject(
                self.geometry["cone"], fcl.Transform(np.array([0.0, 0.0, 5.0]))
            ),
            fcl.CollisionObject(
                self.geometry["cylinder"], fcl.Transform(np.array([0.0, 0.0, -5.0]))
            ),
        ]
        objs3 = [
            fcl.CollisionObject(self.geometry["mesh"]),
            fcl.CollisionObject(
                self.geometry["convex"], fcl.Transform(np.array([0.0, 0.0, -5.0]))
            ),
        ]

        manager1.registerObjects(objs1)
        manager2.registerObjects(objs2)
        manager3.registerObjects(objs3)

        manager1.setup()
        manager2.setup()
        manager3.setup()

        self.assertTrue(len(manager1.getObjects()) == 2)
        self.assertTrue(len(manager2.getObjects()) == 2)
        self.assertTrue(len(manager3.getObjects()) == 2)

        # One-to-many
        o1 = fcl.CollisionObject(self.geometry["box"])
        o2 = fcl.CollisionObject(
            self.geometry["cylinder"], fcl.Transform(np.array([0.0, 0.0, -4.6]))
        )

        cdata = fcl.CollisionData(self.crequest, fcl.CollisionResult())
        manager1.collide(o1, cdata, fcl.defaultCollisionCallback)
        self.assertTrue(cdata.result.is_collision)

        cdata = fcl.CollisionData(self.crequest, fcl.CollisionResult())
        manager1.collide(o2, cdata, fcl.defaultCollisionCallback)
        self.assertFalse(cdata.result.is_collision)

        cdata = fcl.CollisionData(self.crequest, fcl.CollisionResult())
        manager2.collide(o1, cdata, fcl.defaultCollisionCallback)
        self.assertFalse(cdata.result.is_collision)

        cdata = fcl.CollisionData(self.crequest, fcl.CollisionResult())
        manager2.collide(o2, cdata, fcl.defaultCollisionCallback)
        self.assertTrue(cdata.result.is_collision)

        # Many-to-many, internal
        cdata = fcl.CollisionData(self.crequest, fcl.CollisionResult())
        manager1.collide(cdata, fcl.defaultCollisionCallback)
        self.assertTrue(cdata.result.is_collision)

        cdata = fcl.CollisionData(self.crequest, fcl.CollisionResult())
        manager2.collide(cdata, fcl.defaultCollisionCallback)
        self.assertFalse(cdata.result.is_collision)

        # Many-to-many, grouped
        cdata = fcl.CollisionData(self.crequest, fcl.CollisionResult())
        manager1.collide(manager2, cdata, fcl.defaultCollisionCallback)
        self.assertFalse(cdata.result.is_collision)

        cdata = fcl.CollisionData(self.crequest, fcl.CollisionResult())
        manager2.collide(manager3, cdata, fcl.defaultCollisionCallback)
        self.assertTrue(cdata.result.is_collision)

        cdata = fcl.CollisionData(self.crequest, fcl.CollisionResult())
        manager1.collide(manager3, cdata, fcl.defaultCollisionCallback)
        self.assertTrue(cdata.result.is_collision)

    def test_updates(self):
        manager = fcl.DynamicAABBTreeCollisionManager()

        objs = [
            fcl.CollisionObject(self.geometry["sphere"]),
            fcl.CollisionObject(
                self.geometry["sphere"], fcl.Transform(np.array([0.0, 0.0, -5.0]))
            ),
        ]

        manager.registerObjects(objs)
        manager.setup()

        self.assertTrue(len(manager.getObjects()) == 2)

        # Many-to-many, internal
        cdata = fcl.CollisionData(self.crequest, fcl.CollisionResult())
        manager.collide(cdata, fcl.defaultCollisionCallback)
        self.assertFalse(cdata.result.is_collision)

        objs[1].setTranslation(np.array([0.0, 0.0, -0.3]))
        cdata = fcl.CollisionData(self.crequest, fcl.CollisionResult())
        manager.update(objs[1])
        manager.collide(cdata, fcl.defaultCollisionCallback)
        self.assertTrue(cdata.result.is_collision)

    def test_many_objects(self):
        manager = fcl.DynamicAABBTreeCollisionManager()

        objs = [
            fcl.CollisionObject(self.geometry["sphere"]),
            fcl.CollisionObject(
                self.geometry["sphere"], fcl.Transform(np.array([0.0, 0.0, -5.0]))
            ),
            fcl.CollisionObject(
                self.geometry["sphere"], fcl.Transform(np.array([0.0, 0.0, 5.0]))
            ),
        ]

        manager.registerObjects(objs)
        manager.setup()

        self.assertTrue(len(manager.getObjects()) == 3)

        # Many-to-many, internal
        cdata = fcl.CollisionData(self.crequest, fcl.CollisionResult())
        manager.collide(cdata, fcl.defaultCollisionCallback)
        self.assertFalse(cdata.result.is_collision)

        objs[1].setTranslation(np.array([0.0, 0.0, -0.3]))
        manager.update(objs[1])
        cdata = fcl.CollisionData(self.crequest, fcl.CollisionResult())
        manager.collide(cdata, fcl.defaultCollisionCallback)
        self.assertTrue(cdata.result.is_collision)

        objs[1].setTranslation(np.array([0.0, 0.0, -5.0]))
        manager.update(objs[1])
        cdata = fcl.CollisionData(self.crequest, fcl.CollisionResult())
        manager.collide(cdata, fcl.defaultCollisionCallback)
        self.assertFalse(cdata.result.is_collision)

        objs[2].setTranslation(np.array([0.0, 0.0, 0.3]))
        manager.update(objs[2])
        cdata = fcl.CollisionData(self.crequest, fcl.CollisionResult())
        manager.collide(cdata, fcl.defaultCollisionCallback)
        self.assertTrue(cdata.result.is_collision)

        objs[2].setTranslation(np.array([0.0, 0.0, 5.0]))
        manager.update(objs[2])
        cdata = fcl.CollisionData(self.crequest, fcl.CollisionResult())
        manager.collide(cdata, fcl.defaultCollisionCallback)
        self.assertFalse(cdata.result.is_collision)

    def test_managed_distances(self):
        manager1 = fcl.DynamicAABBTreeCollisionManager()
        manager2 = fcl.DynamicAABBTreeCollisionManager()
        manager3 = fcl.DynamicAABBTreeCollisionManager()

        objs1 = [
            fcl.CollisionObject(self.geometry["box"]),
            fcl.CollisionObject(self.geometry["cylinder"]),
        ]
        objs2 = [
            fcl.CollisionObject(
                self.geometry["cone"], fcl.Transform(np.array([0.0, 0.0, 5.0]))
            ),
            fcl.CollisionObject(
                self.geometry["cylinder"], fcl.Transform(np.array([0.0, 0.0, -5.0]))
            ),
        ]
        objs3 = [
            fcl.CollisionObject(self.geometry["mesh"]),
            fcl.CollisionObject(
                self.geometry["convex"], fcl.Transform(np.array([0.0, 0.0, -5.0]))
            ),
        ]

        manager1.registerObjects(objs1)
        manager2.registerObjects(objs2)
        manager3.registerObjects(objs3)

        manager1.setup()
        manager2.setup()
        manager3.setup()

        self.assertTrue(len(manager1.getObjects()) == 2)
        self.assertTrue(len(manager2.getObjects()) == 2)
        self.assertTrue(len(manager3.getObjects()) == 2)

        # One-to-many
        o1 = fcl.CollisionObject(self.geometry["box"])
        o2 = fcl.CollisionObject(
            self.geometry["cylinder"], fcl.Transform(np.array([0.0, 0.0, -4.6]))
        )

        cdata = fcl.DistanceData(self.drequest, fcl.DistanceResult())
        manager1.distance(o1, cdata, fcl.defaultDistanceCallback)
        self.assertTrue(cdata.result.min_distance < 0)

        cdata = fcl.DistanceData(self.drequest, fcl.DistanceResult())
        manager1.distance(o2, cdata, fcl.defaultDistanceCallback)
        self.assertAlmostEqual(cdata.result.min_distance, 3.6, places=6)

        cdata = fcl.DistanceData(self.drequest, fcl.DistanceResult())
        manager2.distance(o1, cdata, fcl.defaultDistanceCallback)
        self.assertAlmostEqual(cdata.result.min_distance, 4.0, places=6)

        cdata = fcl.DistanceData(self.drequest, fcl.DistanceResult())
        manager2.distance(o2, cdata, fcl.defaultDistanceCallback)
        self.assertTrue(cdata.result.min_distance < 0)

        # Many-to-many, internal
        cdata = fcl.DistanceData(self.drequest, fcl.DistanceResult())
        manager1.distance(cdata, fcl.defaultDistanceCallback)
        self.assertTrue(cdata.result.min_distance < 0)

        cdata = fcl.DistanceData(self.drequest, fcl.DistanceResult())
        manager2.distance(cdata, fcl.defaultDistanceCallback)
        self.assertAlmostEqual(cdata.result.min_distance, 9.0, places=6)

        # Many-to-many, grouped
        cdata = fcl.DistanceData(self.drequest, fcl.DistanceResult())
        manager1.distance(manager2, cdata, fcl.defaultDistanceCallback)
        self.assertAlmostEqual(cdata.result.min_distance, 4.0, places=6)

        cdata = fcl.DistanceData(self.drequest, fcl.DistanceResult())
        manager2.distance(manager3, cdata, fcl.defaultDistanceCallback)
        self.assertTrue(cdata.result.min_distance < 0)

        cdata = fcl.DistanceData(self.drequest, fcl.DistanceResult())
        manager1.distance(manager3, cdata, fcl.defaultDistanceCallback)
        self.assertTrue(cdata.result.min_distance < 0)

    # Tests with boxes to make sure nearest points are independent of manager order
    def test_nearest_points(self):
        box1, box1_t = (20.0, 5.0), (-2.0, 0.0)
        box2, box2_t = (5, 5), (-5.0, 10.0)
        box3, box3_t = (5, 5), (-15.0, 15.0)
        q1 = np.array([0.8660254, 0.0, 0.0, 0.5])  # np.pi / 3 z-ax rotation
        q2 = np.array([1.0, 0.0, 0.0, 0.0])  # no rotation

        h = 1000
        g1 = fcl.Box(*box1, h)
        t1 = fcl.Transform(q1, [*box1_t, 0])
        o1 = fcl.CollisionObject(g1, t1)
        g2 = fcl.Box(*box2, h)
        t2 = fcl.Transform(q2, [*box2_t, 0])
        o2 = fcl.CollisionObject(g2, t2)
        g3 = fcl.Box(*box3, h)
        t3 = fcl.Transform(q2, [*box3_t, 0])
        o3 = fcl.CollisionObject(g3, t3)

        manager1 = fcl.DynamicAABBTreeCollisionManager()
        manager1.registerObjects([o1])
        manager1.setup()

        manager2 = fcl.DynamicAABBTreeCollisionManager()
        manager2r = fcl.DynamicAABBTreeCollisionManager()
        manager2.registerObjects([o2, o3])
        manager2r.registerObjects([o3, o2])
        manager2.setup()
        manager2r.setup()

        ddata = fcl.DistanceData(request=fcl.DistanceRequest(enable_nearest_points=True))
        manager1.distance(manager2, ddata, fcl.defaultDistanceCallback)
        self.assertAlmostEqual(
            ddata.result.min_distance,
            np.linalg.norm(np.subtract(*ddata.result.nearest_points)),
            places=6,
        )

        ddata = fcl.DistanceData(request=fcl.DistanceRequest(enable_nearest_points=True))
        manager1.distance(manager2r, ddata, fcl.defaultDistanceCallback)
        self.assertAlmostEqual(
            ddata.result.min_distance,
            np.linalg.norm(np.subtract(*ddata.result.nearest_points)),
            places=6,
        )


if __name__ == "__main__":
    unittest.main()
