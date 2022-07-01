
cdef extern from "<iostream>" namespace "std":
    cdef cppclass istream:
        istream& write(const char*, int) except +

    cdef cppclass iostream(istream):
        iostream() except +

    cdef cppclass stringstream(iostream):
        stringstream() except +
