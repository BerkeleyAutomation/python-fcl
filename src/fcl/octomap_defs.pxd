cimport std_defs as std


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
