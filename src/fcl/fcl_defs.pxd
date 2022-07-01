cimport octomap_defs as octomap
from libcpp cimport bool
from libcpp.memory cimport make_shared, shared_ptr
from libcpp.set cimport set
from libcpp.string cimport string
from libcpp.vector cimport vector


cdef extern from "Python.h":
       ctypedef struct PyObject
       void Py_INCREF(PyObject *obj)
       void Py_DECREF(PyObject *obj)
       object PyObject_CallObject(object obj, object args)
       object PySequence_Concat(object obj1, object obj2)

#cdef extern from "boost/shared_ptr.hpp" namespace "boost":
#    cppclass shared_ptr[T]:
#        shared_ptr() except +
#        shared_ptr(T*) except +
#        T* get()

cdef extern from "fcl/common/types.h" namespace "fcl":
    cdef cppclass Vector3d:
        Vector3d() except +
        Vector3d(double *data) except +
        double& operator[](size_t i)

    cdef cppclass Matrix3d:
        Matrix3d() except +
        Matrix3d(double *data)
        double operator()(size_t i, size_t j)

    cdef cppclass Quaterniond:
        Quaterniond() except +
        Quaterniond(double a, double b, double c, double d) except +
        Quaterniond(Matrix3d& R) except +
        double& w()
        double& x()
        double& y()
        double& z()
        Matrix3d& toRotationMatrix()

    cdef cppclass Transform3d:
        Transform3d() except +
        Transform3d(Transform3d& tf_)
        void setIdentity()
        Matrix3d& linear()
        Vector3d& translation()

cdef extern from "fcl/narrowphase/continuous_collision_request.h" namespace "fcl":
    cdef enum CCDMotionType:
        CCDM_TRANS, CCDM_LINEAR, CCDM_SCREW, CCDM_SPLINE

    cdef enum CCDSolverType:
        CCDC_NAIVE, CCDC_CONSERVATIVE_ADVANCEMENT, CCDC_RAY_SHOOTING, CCDC_POLYNOMIAL_SOLVER

    cdef cppclass ContinuousCollisionRequestd:
         size_t num_max_iterations_,
         double toc_err_,
         CCDMotionType ccd_motion_type_,
         GJKSolverType gjk_solver_type_,
         GJKSolverType ccd_solver_type_

         ContinuousCollisionRequestd(
                             size_t num_max_iterations_,
                             double toc_err_,
                             CCDMotionType ccd_motion_type_,
                             GJKSolverType gjk_solver_type_,
                             CCDSolverType ccd_solver_type_ )

cdef extern from "fcl/narrowphase/gjk_solver_type.h" namespace "fcl":
    cdef enum GJKSolverType:
        GST_LIBCCD, GST_INDEP

cdef extern from "fcl/narrowphase/contact.h" namespace "fcl":
    cdef cppclass Contactd:
        CollisionGeometryd *o1
        CollisionGeometryd *o2
        int b1
        int b2
        Vector3d normal
        Vector3d pos
        double penetration_depth
        Contactd() except +
        Contactd(CollisionGeometryd* o1_,
                CollisionGeometryd* o2_,
                int b1_, int b2_) except +

cdef extern from "fcl/narrowphase/cost_source.h" namespace "fcl":
    cdef cppclass CostSourced:
        Vector3d aabb_min
        Vector3d aabb_max
        double cost_density
        double total_cost

cdef extern from "fcl/narrowphase/collision_result.h" namespace "fcl":
    cdef cppclass CollisionResultd:
        CollisionResultd() except +
        bool isCollision()
        void getContacts(vector[Contactd]& contacts_)
        void getCostSources(vector[CostSourced]& cost_sources_)

cdef extern from "fcl/narrowphase/continuous_collision_result.h" namespace "fcl":
    cdef cppclass ContinuousCollisionResultd:
        ContinuousCollisionResultd() except +
        bool is_collide
        double time_of_contact
        Transform3d contact_tf1, contact_tf2

cdef extern from "fcl/narrowphase/collision_request.h" namespace "fcl":
    cdef cppclass CollisionRequestd:
        size_t num_max_contacts
        bool enable_contact
        size_t num_max_cost_sources
        bool enable_cost
        bool use_approximate_cost
        GJKSolverType gjk_solver_type
        CollisionRequestd(size_t num_max_contacts_,
                         bool enable_contact_,
                         size_t num_max_cost_sources_,
                         bool enable_cost_,
                         bool use_approximate_cost_,
                         GJKSolverType gjk_solver_type_)

cdef extern from "fcl/narrowphase/distance_result.h" namespace "fcl":
    cdef cppclass DistanceResultd:
        double min_distance
        Vector3d* nearest_points
        CollisionGeometryd* o1
        CollisionGeometryd* o2
        int b1
        int b2
        DistanceResultd(double min_distance_) except +
        DistanceResultd() except +

cdef extern from "fcl/narrowphase/distance_request.h" namespace "fcl":
    cdef cppclass DistanceRequestd:
        bool enable_nearest_points
        bool enable_signed_distance
        GJKSolverType gjk_solver_type
        DistanceRequestd(bool enable_nearest_points_, bool enable_signed_distance, GJKSolverType gjk_solver_type_) except +

cdef extern from "fcl/geometry/collision_geometry.h" namespace "fcl":
    cdef enum OBJECT_TYPE:
        OT_UNKNOWN, OT_BVH, OT_GEOM, OT_OCTREE, OT_COUNT
    cdef enum NODE_TYPE:
        BV_UNKNOWN, BV_AABB, BV_OBB, BV_RSS, BV_kIOS, BV_OBBRSS, BV_KDOP16, BV_KDOP18, BV_KDOP24,
        GEOM_BOX, GEOM_SPHERE, GEOM_ELLIPSOID, GEOM_CAPSULE, GEOM_CONE, GEOM_CYLINDER, GEOM_CONVEX,
        GEOM_PLANE, GEOM_HALFSPACE, GEOM_TRIANGLE, GEOM_OCTREE, NODE_COUNT

    cdef cppclass CollisionGeometryd:
        CollisionGeometryd() except +
        OBJECT_TYPE getObjectType()
        NODE_TYPE getNodeType()
        void computeLocalAABB()
        Vector3d aabb_center
        double aabb_radius
        double cost_density
        double threshold_occupied
        double threshold_free

    ctypedef CollisionGeometryd const_CollisionGeometryd "const fcl::CollisionGeometryd"

cdef extern from "fcl/narrowphase/collision_object.h" namespace "fcl":
    cdef cppclass CollisionObjectd:
        CollisionObjectd() except +
        CollisionObjectd(shared_ptr[CollisionGeometryd]& cgeom_) except +
        CollisionObjectd(shared_ptr[CollisionGeometryd]& cgeom_, Transform3d& tf) except +
        OBJECT_TYPE getObjectType()
        NODE_TYPE getNodeType()
        Vector3d& getTranslation()
        Matrix3d& getRotation()
        Quaterniond& getQuatRotation()
        Transform3d& getTransform()
        CollisionGeometryd* getCollisionGeometry()
        void setTranslation(Vector3d& T)
        void setRotation(Matrix3d& M)
        void setQuatRotation(Quaterniond& q)
        void setTransform(Quaterniond& q, Vector3d& T)
        void setTransform(Matrix3d& q, Vector3d& T)
        void setTransform(Transform3d& tf)
        void setUserData(void *data)
        void computeAABB()
        void *getUserData()
        bool isOccupied()
        bool isFree()
        bool isUncertain()

    ctypedef CollisionObjectd const_CollisionObjectd "const fcl::CollisionObjectd"

cdef extern from "fcl/geometry/shape/shape_base.h" namespace "fcl":
    cdef cppclass ShapeBased(CollisionGeometryd):
        ShapeBased() except +

cdef extern from "fcl/geometry/shape/triangle_p.h" namespace "fcl":
    cdef cppclass TrianglePd(ShapeBased):
        TrianglePd(Vector3d& a_, Vector3d& b_, Vector3d& c_) except +
        Vector3d a, b, c

cdef extern from "fcl/geometry/shape/box.h" namespace "fcl":
    cdef cppclass Boxd(ShapeBased):
        Boxd(double x, double y, double z) except +
        Vector3d side

cdef extern from "fcl/geometry/shape/sphere.h" namespace "fcl":
    cdef cppclass Sphered(ShapeBased):
        Sphered(double radius_) except +
        double radius

cdef extern from "fcl/geometry/shape/ellipsoid.h" namespace "fcl":
    cdef cppclass Ellipsoidd(ShapeBased):
        Ellipsoidd(double a_, double b_, double c_) except +
        Vector3d radii

cdef extern from "fcl/geometry/shape/capsule.h" namespace "fcl":
    cdef cppclass Capsuled(ShapeBased):
        Capsuled(double radius_, double lz_) except +
        double radius
        double lz

cdef extern from "fcl/geometry/shape/cone.h" namespace "fcl":
    cdef cppclass Coned(ShapeBased):
        Coned(double radius_, double lz_) except +
        double radius
        double lz

cdef extern from "fcl/geometry/shape/cylinder.h" namespace "fcl":
    cdef cppclass Cylinderd(ShapeBased):
        Cylinderd(double radius_, double lz_) except +
        double radius
        double lz

cdef extern from "fcl/geometry/shape/convex.h" namespace "fcl":
    cdef cppclass Convexd(ShapeBased):
        Convexd(const shared_ptr[const vector[Vector3d]]& vertices, int num_faces, const shared_ptr[const vector[int]]& faces) except +
        int getFaceCount()

cdef extern from "fcl/geometry/shape/halfspace.h" namespace "fcl":
    cdef cppclass Halfspaced(ShapeBased):
        Halfspaced(Vector3d& n_, double d_) except +
        Vector3d n
        double d

cdef extern from "fcl/geometry/shape/plane.h" namespace "fcl":
    cdef cppclass Planed(ShapeBased):
        Planed(Vector3d& n_, double d_) except +
        Vector3d n
        double d

cdef extern from "fcl/broadphase/broadphase_collision_manager.h" namespace "fcl":
    ctypedef bool (*CollisionCallBack)(CollisionObjectd* o1, CollisionObjectd* o2, void* cdata)
    ctypedef bool (*DistanceCallBack)(CollisionObjectd* o1, CollisionObjectd* o2, void* cdata, double& dist)

cdef extern from "fcl/broadphase/broadphase_dynamic_AABB_tree.h" namespace "fcl":
    cdef cppclass DynamicAABBTreeCollisionManagerd:
        DynamicAABBTreeCollisionManagerd() except +
        void registerObjects(vector[CollisionObjectd*]& other_objs)
        void registerObject(CollisionObjectd* obj)
        void unregisterObject(CollisionObjectd* obj)
        void collide(DynamicAABBTreeCollisionManagerd* mgr, void* cdata, CollisionCallBack callback)
        void distance(DynamicAABBTreeCollisionManagerd* mgr, void* cdata, DistanceCallBack callback)
        void collide(CollisionObjectd* obj, void* cdata, CollisionCallBack callback)
        void distance(CollisionObjectd* obj, void* cdata, DistanceCallBack callback)
        void collide(void* cdata, CollisionCallBack callback)
        void distance(void* cdata, DistanceCallBack callback)
        void setup()
        void update()
        void update(CollisionObjectd* updated_obj)
        void update(vector[CollisionObjectd*] updated_objs)
        void clear()
        bool empty()
        size_t size()
        int max_tree_nonbalanced_level
        int tree_incremental_balance_pass
        int& tree_topdown_balance_threshold
        int& tree_topdown_level
        int tree_init_level
        bool octree_as_geometry_collide
        bool octree_as_geometry_distance

cdef extern from "fcl/narrowphase/collision.h" namespace "fcl":
    size_t collide(CollisionObjectd* o1, CollisionObjectd* o2,
                   CollisionRequestd& request,
                   CollisionResultd& result)

    size_t collide(CollisionGeometryd* o1, Transform3d& tf1,
                   CollisionGeometryd* o2, Transform3d& tf2,
                   CollisionRequestd& request,
                   CollisionResultd& result)

cdef extern from "fcl/narrowphase/continuous_collision.h" namespace "fcl":
    double continuousCollide(CollisionGeometryd* o1, Transform3d& tf1_beg, Transform3d& tf1_end,
                               CollisionGeometryd* o2, Transform3d& tf2_beg, Transform3d& tf2_end,
                               ContinuousCollisionRequestd& request,
                               ContinuousCollisionResultd& result)

    double continuousCollide(CollisionObjectd* o1, Transform3d& tf1_end,
                               CollisionObjectd* o2, Transform3d& tf2_end,
                               ContinuousCollisionRequestd& request,
                               ContinuousCollisionResultd& result)


cdef extern from "fcl/narrowphase/distance.h" namespace "fcl":
    double distance(CollisionObjectd* o1, CollisionObjectd* o2,
                      DistanceRequestd& request, DistanceResultd& result)
    double distance(CollisionGeometryd* o1, Transform3d& tf1,
                      CollisionGeometryd* o2, Transform3d& tf2,
                      DistanceRequestd& request, DistanceResultd& result)

cdef extern from "fcl/geometry/bvh/BVH_internal.h" namespace "fcl":
    cdef enum BVHModelType:
        BVH_MODEL_UNKNOWN,   # unknown model type
        BVH_MODEL_TRIANGLES, # triangle model
        BVH_MODEL_POINTCLOUD # point cloud model

    cdef enum  BVHReturnCode:
        BVH_OK = 0,                              # BVH is valid
        BVH_ERR_MODEL_OUT_OF_MEMORY = -1,        # Cannot allocate memory for vertices and triangles
        BVH_ERR_BUILD_OUT_OF_SEQUENCE = -2,      # BVH construction does not follow correct sequence
        BVH_ERR_BUILD_EMPTY_MODEL = -3,          # BVH geometry is not prepared
        BVH_ERR_BUILD_EMPTY_PREVIOUS_FRAME = -4, # BVH geometry in previous frame is not prepared
        BVH_ERR_UNSUPPORTED_FUNCTION = -5,       # BVH funtion is not supported
        BVH_ERR_UNUPDATED_MODEL = -6,            # BVH model update failed
        BVH_ERR_INCORRECT_DATA = -7,             # BVH data is not valid
        BVH_ERR_UNKNOWN = -8                     # Unknown failure


    cdef enum BVHBuildState:
        BVH_BUILD_STATE_EMPTY,         # empty state, immediately after constructor
        BVH_BUILD_STATE_BEGUN,         # after beginModel(), state for adding geometry primitives
        BVH_BUILD_STATE_PROCESSED,     # after tree has been build, ready for cd use
        BVH_BUILD_STATE_UPDATE_BEGUN,  # after beginUpdateModel(), state for updating geometry primitives
        BVH_BUILD_STATE_UPDATED,       # after tree has been build for updated geometry, ready for ccd use
        BVH_BUILD_STATE_REPLACE_BEGUN, # after beginReplaceModel(), state for replacing geometry primitives

cdef extern from "fcl/math/triangle.h" namespace "fcl":
    cdef cppclass Triangle:
        Triangle() except +
        Triangle(size_t p1, size_t p2, size_t p3) except +
        size_t vids[3]

# TODO what about these guys?
cdef extern from "fcl/geometry/bvh/detail/BV_splitter_base.h" namespace "fcl":
    cdef cppclass BVSplitterBase:
        pass

cdef extern from "fcl/geometry/bvh/detail/BV_fitter_base.h" namespace "fcl":
    cdef cppclass BVFitterBase:
        pass

cdef extern from "fcl/geometry/bvh/BVH_model.h" namespace "fcl":
    # Cython only accepts type template parameters.
    # see https://groups.google.com/forum/#!topic/cython-users/xAZxdCFw6Xs
    cdef cppclass BVHModel "fcl::BVHModel<fcl::OBBRSSd>" ( CollisionGeometryd ):
        # Constructing an empty BVH
        BVHModel() except +
        BVHModel(BVHModel& other) except +
        #
        #Geometry point data
        Vector3d* vertices
        #
        #Geometry triangle index data, will be NULL for point clouds
        Triangle* tri_indices
        #
        #Geometry point data in previous frame
        Vector3d* prev_vertices
        #
        #Number of triangles
        int num_tris
        #
        #Number of points
        int num_vertices
        #
        #The state of BVH building process
        BVHBuildState build_state
        #
        # # #Split rule to split one BV node into two children
        #
        # boost::shared_ptr<BVSplitterBase<BV> > bv_splitter
        shared_ptr[BVSplitterBase] bv_splitter
        # boost::shared_ptr<BVFitterBase<BV> > bv_fitter
        shared_ptr[BVFitterBase] bv_fitter

        int beginModel(int num_tris_, int num_vertices_)

        int addVertex(const Vector3d& p)

        int addTriangle(const Vector3d& p1, const Vector3d& p2, const Vector3d& p3)

        #int addSubModel(const std::vector<Vector3d>& ps)
        # void getCostSources(vector[CostSourced]& cost_sources_)

        #int addSubModel(const vector[Vector3d]& ps)
        #
        int addSubModel(const vector[Vector3d]& ps, const vector[Triangle]& ts)

        int endModel()

        int buildTree()

        # void computeLocalAABB()


cdef extern from "fcl/geometry/octree/octree.h" namespace "fcl":
    cdef cppclass OcTreed(CollisionGeometryd):
        # Constructing
        OcTreed(double resolution) except +
        OcTreed(shared_ptr[octomap.OcTree]& tree_) except +
