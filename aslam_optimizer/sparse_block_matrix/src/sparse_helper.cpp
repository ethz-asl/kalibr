#include <sparse_block_matrix/sparse_helper.h>



using namespace std;

namespace sparse_block_matrix {



bool writeVector(const char* filename, const double*v, int n)
{
	ofstream os(filename);
	os << fixed;
	for (int i=0; i<n; i++)
		os << *v++ << endl;
	return os.good();
}


} // end namespace
