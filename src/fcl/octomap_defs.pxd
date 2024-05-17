cimport std_defs as std
from libcpp cimport bool

cdef extern from "octomap/math/Vector3.h" namespace "octomath":
    cdef cppclass Vector3:
        Vector3() except +
        Vector3(float, float, float) except +
        Vector3(Vector3& other) except +
        float& x()
        float& y()
        float& z()

cdef extern from "octomap/octomap_types.h" namespace "octomap":
    ctypedef Vector3 point3d

cdef extern from "octomap/Pointcloud.h" namespace "octomap":
    cdef cppclass Pointcloud:
        Pointcloud() except +
        void push_back(float, float, float)
        void push_back(point3d* p)


cdef extern from "octomap/OccupancyOcTreeBase.h" namespace "octomap":
    # Cython only accepts type template parameters.
    # see https://groups.google.com/forum/#!topic/cython-users/xAZxdCFw6Xs
    cdef cppclass OccupancyOcTreeBase "octomap::OccupancyOcTreeBase<octomap::OcTreeNode>" :
        # Constructing
        OccupancyOcTreeBase(double resolution) except +

        #  Reads only the data (=complete tree structure) from the input stream.
        #  The tree needs to be constructed with the proper header information
        #  beforehand, see readBinary().
        std.istream& readBinaryData(std.istream &s) except +

cdef extern from "octomap/OcTree.h" namespace "octomap":
    cdef cppclass OcTree(OccupancyOcTreeBase):
        # Constructing
        OcTree(double resolution) except +

        void insertPointCloud(Pointcloud& scan, point3d& sensor_origin,
                              double maxrange, bool lazy_eval, bool discretize) except +


