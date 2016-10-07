// g2o - General Graph Optimization
// Copyright (C) 2011 R. Kuemmerle, G. Grisetti, W. Burgard
// 
// g2o is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published
// by the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
// 
// g2o is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
// 
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.


namespace sparse_block_matrix {

    
    
    
    template<typename T>
    bool writeCCSMatrix(const char* filename, int rows, int cols, const T* Ap, const T* Ai, const double* Ax, bool upperTriangleSymmetric)
    {
	using namespace std;
        vector<TripletEntry> entries;
        entries.reserve(Ap[cols]);
        for (int i=0; i < cols; i++) {
            const int& rbeg = Ap[i];
            const int& rend = Ap[i+1];
            for (int j = rbeg; j < rend; j++) {
                entries.push_back(TripletEntry(Ai[j], i, Ax[j]));
                if (upperTriangleSymmetric && Ai[j] != i)
                    entries.push_back(TripletEntry(i, Ai[j], Ax[j]));
            }
        }
        sort(entries.begin(), entries.end(), TripletColSort());
        
        string name = filename;
        std::string::size_type lastDot = name.find_last_of('.');
        if (lastDot != std::string::npos)
            name = name.substr(0, lastDot);
        
        std::ofstream fout(filename);
        fout << "# name: " << name << std::endl;
        fout << "# type: sparse matrix" << std::endl;
        fout << "# nnz: " << entries.size() << std::endl;
        fout << "# rows: " << rows << std::endl;
        fout << "# columns: " << cols << std::endl;
        //fout << fixed;
        fout << setprecision(9) << endl;
        for (vector<TripletEntry>::const_iterator it = entries.begin(); it != entries.end(); ++it) {
            const TripletEntry& entry = *it;
            fout << entry.r+1 << " " << entry.c+1 << " " << entry.x << std::endl;
        }
        return fout.good();
    }
    
    
    
    
} // end namespace

