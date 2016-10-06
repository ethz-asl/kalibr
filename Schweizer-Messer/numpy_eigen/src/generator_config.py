MaximalStaticDimension = 6
BuildFast = False


if BuildFast:
    MaximalStaticDimension = 1

dims = ['Eigen::Dynamic']
dimTags = ['D']
for i in range(1, MaximalStaticDimension + 1):
    dims.append(str(i))
    dimTags.append(str(i))

#types =    ['char','short','int','long','unsigned char', 'unsigned short', 'unsigned int', 'unsigned long', 'float', 'double', 'std::complex<float>','std::complex<double>']
#typeTags = ['char','short','int','long','uchar', 'ushort', 'uint', 'ulong', 'float', 'double', 'cfloat','cdouble']
types = ['int', 'float', 'double', 'boost::uint8_t', 'boost::int64_t']
typeTags = ['int', 'float', 'double', 'uchar', 'long']

if BuildFast:
    types = ['double']
    typeTags = ['double']
