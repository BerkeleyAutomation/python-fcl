from .fcl import *


class Transform(Transform):
    def __init__(self, *args):
        super().__init__()
        self.args = args

    def __reduce__(self):
        return self.__class__, self.args


class CollisionObject(CollisionObject):
    def __init__(self, geom=None, tf=None, _no_instance=False):
        super().__init__()
        self.geom = geom
        self.tf = tf
        self._no_instance = _no_instance

    def __reduce__(self):
        return self.__class__, (self.geom, self.tf, self._no_instance)
    

class BVHModel(BVHModel):
    def __init__(self, num_tris=0, num_vertices=0, verts=None, triangles=None):
        super().__init__()

        self.num_tris = num_tris
        self.num_vertices = num_vertices
        self.verts = verts
        self.triangles = triangles

        if num_tris != 0 and num_vertices != 0 and not verts is None and not triangles is None:
            self.beginModel(num_tris, num_vertices)
            self.addSubModel(verts, triangles)
            self.endModel()


    def beginModel(self, num_tris_=0, num_vertices_=0):
        super().beginModel(num_tris_, num_vertices_)
        self.num_tris = num_tris_
        self.num_vertices = num_vertices_

    def addSubModel(self, verts, triangles):
        super().addSubModel(verts, triangles)
        self.verts = verts.copy()
        self.triangles = triangles.copy()

    def __reduce__(self):
        return self.__class__, (self.num_tris, self.num_vertices, self.verts, self.triangles)
