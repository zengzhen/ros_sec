/**
 * \file        similarityMeasure.cpp
 * \author      Zhen Zeng (zengzhen@umich.edu) 
 */

#include "ros_sec/SEC/similarityMeasure.h"

namespace TableObject{

    similarityMeasure::similarityMeasure(const eventChain& sec1, const eventChain& sec2)
    {
        _sec1 = sec1;
        _sec2 = sec2;
        _spatial_equal_threshold = 60;
    }
    
    void similarityMeasure::setSpatialThreshold(float spatial_threshold)
    {
        _spatial_threshold = spatial_threshold*100;
    }
    
    void similarityMeasure::setTemporalThreshold(float temporal_threshold)
    {
        _temporal_threshold = temporal_threshold*100;
    }

    float similarityMeasure::spatialSimilarity()
    {
        sec comSec1, comSec2;
        _sec1.getCompressedSec(comSec1);
        _sec2.getCompressedSec(comSec2);
        
        // calculate similarity measure for each row of derSec1 vs. each row of derSec2
        std::cout << "calculating spatial similarity (threshold=" << _spatial_threshold << ")" << std::endl;
        float ss = 0;
        for(int i=0; i<comSec1.row; i++)
        {
            float ss_i = 0;
            std::vector<float> ss_i_vector;
            std::cout << "ss[" << i << "][:] = ";
            for(int j=0; j<comSec2.row; j++)
            {
                float ss_ij = subString(comSec1.event_chain[i], comSec2.event_chain[j], "spatial");
                if(ss_ij > ss_i) ss_i = ss_ij;
                ss_i_vector.push_back(ss_ij);
            }
            std:: cout << "max_f = " << ss_i << std::endl; 
            
            ss = ss + ss_i;
            _ss.push_back(ss_i_vector);
        }
        
        // calculate total spatial similarity
        ss = ss / comSec1.row;
        std::cout << std::endl;
        std::cout << "spatial simlarity = " << ss << std::endl;
        
        return ss;
    }

    float similarityMeasure::temporalSimilarity(std::vector<int>& add_row_index_sec2, 
                                                std::vector<int>& matched_row_index_sec1, 
                                                std::vector<int>& matched_col_index_sec1,
                                                std::vector<int>& matched_col_index_sec2)
    {
        sec derSec1, derSec2;
        _sec1.getDerivativeSec(derSec1);
        _sec2.getDerivativeSec(derSec2);
        
        // find all pairs of matches between derSec1, derSec2
        // store in match_map ( # row <= # col )
        std::vector<std::vector<int>> match_map;
        if(derSec1.row <= derSec2.row)
        {
            // match_map has # of row = # good permutations of the event chain with more rows
            for(int i=0; i<derSec1.row; i++)
            {
                std::vector<int> match_idx;
                float similarity = 0;
                // find the best match similarity for each row of sec1
                for(int j=0; j<derSec2.row; j++)
                {
                    if(_ss[i][j]>similarity) similarity = _ss[i][j];
                }
                // find all the best matches for each row of sec1
                /*if(similarity < _spatial_equal_threshold)
                {
                    match_idx.push_back(-1);
                }else*/{
                    for(int j=0; j<derSec2.row; j++)
                    {
                        if(_ss[i][j]==similarity)
                        {
                            match_idx.push_back(j);
                        }
                    }
                }
                match_map.push_back(match_idx);
            }
            
            for(int i=0; i<derSec2.row; i++)
            {
                float similarity = 0;
                // find the best match similarity for each row of sec2
                for(int j=0; j<derSec1.row; j++)
                {
                    if(_ss[j][i]>similarity) similarity = _ss[j][i];
                }
                // find all the best matches for each row of sec1
                if(similarity < _spatial_equal_threshold)
                {
                    add_row_index_sec2.push_back(i);
                }
            }
        }else{
            // match_map has # of row = # of good permutations of the event chain with more rows
            for(int i=0; i<derSec2.row; i++)
            {
                std::vector<int> match_idx;
                float similarity = 0;
                // find the best match similarity for each row of sec2
                for(int j=0; j<derSec1.row; j++)
                {
                    if(_ss[j][i]>similarity) similarity = _ss[j][i];
                }
                // find all the best matches for each row of sec2
                /*if(similarity < _spatial_equal_threshold)
                {
                    match_idx.push_back(-1);
                    add_row_index_sec2.push_back(i);
                }else*/{
                    for(int j=0; j<derSec1.row; j++)
                    {
                        if(_ss[j][i]==similarity)
                        {
                            match_idx.push_back(j);
                        }
                    }
                }
                match_map.push_back(match_idx);
            }
        }
        
        // display the matching
        std::cout << "match_map: " << std::endl;
        for(int i=0; i<match_map.size(); i++)
        {
            for(int j=0;j<match_map[i].size(); j++)
            {
                std::cout << match_map[i][j] << " ";
            }
            std::cout << std::endl;
        }
        
//         // contruct test match_map
//         match_map.clear();
//         int row1[] = {0}; std::vector<int> map1 (row1, row1 + sizeof(row1) / sizeof(int) ); match_map.push_back(map1);
//         int row2[] = {0}; std::vector<int> map2 (row2, row2 + sizeof(row2) / sizeof(int) ); match_map.push_back(map2);
//         int row3[] = {1,2}; std::vector<int> map3 (row3, row3 + sizeof(row3) / sizeof(int) ); match_map.push_back(map3);
//         int row4[] = {3,4}; std::vector<int> map4 (row4, row4 + sizeof(row4) / sizeof(int) ); match_map.push_back(map4);
        
        // find all possible permutations
        std::vector<std::vector<int>> permutation;
        std::vector<int> temp_permutation;
        permute(match_map, permutation, temp_permutation);
        
        // display permutations
        std::cout << "before completion, permutations: " << std::endl;
        for(int i=0; i<permutation.size(); i++)
        {
            for(int j=0; j<permutation[i].size(); j++)
            {
                std::cout << permutation[i][j] << " ";
            }
            std::cout << std::endl;
        }
        
        // complete the permutation
        int max_row_num = std::max(derSec1.row, derSec2.row);
        std::vector<int> rows_to_be_removed;
        int size_before_complete = permutation.size();
        for(int i=0; i<size_before_complete; i++)
        {
            std::vector<bool> check_permutation(max_row_num, false);
            for(int j=0;j<permutation[i].size(); j++)
            {
                if(permutation[i][j] != -1) check_permutation[permutation[i][j]]=true;
            }
            std::vector<int> rows_to_be_shuffled;
            for(int j=0; j<max_row_num; j++)
            {
                if(!check_permutation[j]) rows_to_be_shuffled.push_back(j);
            }
            
            if(rows_to_be_shuffled.size()>0)
            {
                std::sort (rows_to_be_shuffled.begin(),rows_to_be_shuffled.begin()+rows_to_be_shuffled.size());
                std::reverse (rows_to_be_shuffled.begin(),rows_to_be_shuffled.begin()+rows_to_be_shuffled.size());
                rows_to_be_removed.push_back(i);
                do {
//                     for(int j=0; j<rows_to_be_shuffled.size(); j++) std::cout << rows_to_be_shuffled[j] << " ";
//                     std::cout << std::endl;
                    
                    std::vector<int> one_shuffle;
                    for(int j=0; j<rows_to_be_shuffled.size(); j++) one_shuffle.push_back(rows_to_be_shuffled[j]);
                    // fill -1 in permutation row first, then append with rest of the shuffle
                    std::vector<int> one_permutation;
                    int count = 0;
                    for(int j=0;j<permutation[i].size(); j++)
                    {
                        if(permutation[i][j]==-1) 
                        {
                            one_permutation.push_back(one_shuffle[count]);
                            count = count + 1;
                        }else{
                            one_permutation.push_back(permutation[i][j]);
                        }
                    }
                    for(int j=count;j<one_shuffle.size(); j++) one_permutation.push_back(one_shuffle[j]);
                    permutation.push_back(one_permutation);
                    
                } while ( std::prev_permutation(rows_to_be_shuffled.begin(),rows_to_be_shuffled.begin()+rows_to_be_shuffled.size()) );
            }
        }
        for(int i=rows_to_be_removed.size()-1; i>=0; i--)
        {
            permutation.erase(permutation.begin()+rows_to_be_removed[i]);
        }
        
        // display permutations
        std::cout << "after completion, permutations: " << std::endl;
        for(int i=0; i<permutation.size(); i++)
        {
            for(int j=0; j<permutation[i].size(); j++)
            {
                std::cout << permutation[i][j] << " ";
            }
            std::cout << std::endl;
        }
        
//         sec temp_derSec1; std::memcpy(&temp_derSec1, &derSec2, sizeof(derSec2));
//         std::memcpy(&derSec2, &derSec1, sizeof(derSec1));
//         std::memcpy(&derSec1, &temp_derSec1, sizeof(temp_derSec1));
        
        // construct premutations
        std::cout << "calculating temporal similarity (threshold=" << _temporal_threshold << ")" << std::endl;
        std::vector<sec> permutated_derSec1_seq;
        std::vector<sec> permutated_derSec2_seq;
        if(derSec1.row <= derSec2.row)
        {
            derSec1.display();
            for(int p=0; p<permutation.size(); p++)
            {
                // construct the derivative sec for each permutation
                sec permutated_derSec2;
                permutated_derSec2.row = 0;
                for(int i=0; i<permutation[p].size(); i++)
                {
                    std::vector<std::string> permutated_row;
                    // fill dummy entries for short col
                    if(permutation[p][i]==-1)
                    {
                        for(int j=0; j<derSec2.event_chain[0].size(); j++) permutated_row.push_back("dummy");
                    }else{
                        permutated_row = derSec2.event_chain[permutation[p][i]];
                    }
                    permutated_derSec2.event_chain.push_back(permutated_row);
                    permutated_derSec2.cols.push_back(permutated_row.size());
                    permutated_derSec2.row = permutated_derSec2.row + 1;
                }
                std::cout << "permutation " << p << ":" << std::endl;
                permutated_derSec2.display();
                permutated_derSec2_seq.push_back(permutated_derSec2);
            }
            permutated_derSec1_seq.push_back(derSec1);
        }else{
            derSec2.display();
            for(int p=0; p<permutation.size(); p++)
            {
                // construct the derivative sec for each permutation
                sec permutated_derSec1;
                permutated_derSec1.row = 0;
                for(int i=0; i<permutation[p].size(); i++)
                {
                    std::vector<std::string> permutated_row;
                    // fill dummy entries for short col
                    if(permutation[p][i]==-1)
                    {
                        for(int j=0; j<derSec1.event_chain[0].size(); j++) permutated_row.push_back("dummy");
                    }else{
                        permutated_row = derSec1.event_chain[permutation[p][i]];
                    }
                    permutated_derSec1.event_chain.push_back(permutated_row);
                    permutated_derSec1.cols.push_back(permutated_row.size());
                    permutated_derSec1.row = permutated_derSec1.row + 1;
                }
                std::cout << "permutation " << p << ":" << std::endl;
                permutated_derSec1.display();
                permutated_derSec1_seq.push_back(permutated_derSec1);
            }
            permutated_derSec2_seq.push_back(derSec2);
        }
        
         // for each permutation, find the similarity matrix
         // permutated_derSec1_seq vs permutated_derSec2_seq
        float best_ts = 0;
        int best_ts_permutation_idx=-1;
        for(int p=0; p<permutation.size(); p++)
        {
            _ts.clear();
            
            std::cout << "permutation " << p << ":" << std::endl;
            sec newDerSec1, newDerSec2;
            if(derSec1.row <= derSec2.row) 
            {
                newDerSec1 = permutated_derSec1_seq[0];
                newDerSec2 = permutated_derSec2_seq[p];
            }else{
                newDerSec1 = permutated_derSec1_seq[p];
                newDerSec2 = permutated_derSec2_seq[0];
            }
            
            for(int i=0; i<newDerSec1.cols[0]; i++)
            {
                float ts_i = 0;
                std::vector<float> ts_i_vector;
                std::cout << "ts[" << i << "][:] = ";
                for(int j=0; j<newDerSec2.cols[0]; j++)
                {
                    // extract the cols from each newDerSec
                    std::vector<std::string> col1; 
                    std::vector<std::string> col2;
                    for(int row_i=0; row_i<newDerSec1.row; row_i++) col1.push_back(newDerSec1.event_chain[row_i][i]);
                    for(int row_i=0; row_i<newDerSec2.row; row_i++) col2.push_back(newDerSec2.event_chain[row_i][j]);
                    
                    // calculate the similarity between col1 and col2
                    float ts_ij = subString( col1, col2, "temporal");
                    if(ts_ij > ts_i) ts_i = ts_ij;
                    ts_i_vector.push_back(ts_ij);
                }
                std:: cout << "max_f = " << ts_i << std::endl; 
                
                _ts.push_back(ts_i_vector);
            }
            
            // find the longest similar subsequence given the temporal similarity matrix _ts
            int lss_length;
            std::vector<std::vector<std::string>> back_pointers;
            lss(lss_length, back_pointers);
            
            // extract the lss
            float ts = 0;
            std::vector<std::vector<int>> LSS;
            int i=_ts.size(), j=_ts[0].size();
            while(i!=0 & j!=0)
            {
                if(std::strcmp(back_pointers[i][j].c_str(), "addxy")==0)
                {
                    std::vector<int> LSS_row;
                    LSS_row.push_back(i-1); LSS_row.push_back(j-1);
                    LSS.push_back(LSS_row);
                    
                    ts = ts + _ts[i-1][j-1];
                    
                    i--; j--;
                }else if(std::strcmp(back_pointers[i][j].c_str(), "skipx")==0){
                    i--;
                }else if(std::strcmp(back_pointers[i][j].c_str(), "skipy")==0){
                    j--;
                }
            }
            
//             // display back pointers
//             for(int i=0; i<=_ts.size(); i++)
//             {
//                 for(int j=0; j<=_ts[0].size(); j++)
//                 {
//                     std::cout << back_pointers[i][j].c_str() << " ";
//                 }
//                 std::cout << std::endl;
//             }
//             
//             // display LSS
//             for(int i=0; i<LSS.size(); i++){
//                 for(int j=0; j<LSS[i].size(); j++) {
//                     std::cout << LSS[i][j] << " ";
//                 }
//                 std::cout << std::endl;
//             }
            
            // calculate mean temporal similarity
//             sec comSec1, comSec2;
//             _sec1.getCompressedSec(comSec1);
//             _sec2.getCompressedSec(comSec2);
            ts = ts / std::min(_ts.size(), _ts[0].size());
            if( ts > best_ts)
            {
                best_ts = ts;
                best_ts_permutation_idx = p;
                _best_back_pointers = back_pointers;
            }
            std::cout << "lss temporal simlarity = " << ts << std::endl;
            ts = ts * std::min(_ts.size(), _ts[0].size()) / std::max(_ts.size(), _ts[0].size());
            std::cout << "lss temporal simlarity (subactivity?) = " << ts << std::endl;
        }
        std::cout << "best temporal similarity = " << best_ts << std::endl;
        
        if(best_ts_permutation_idx != -1)
        {
            // matched row index in sec1 (needed in learning stage)
            if(derSec1.row <= derSec2.row)
            {
                for(int i=0; i<permutation[best_ts_permutation_idx].size(); i++)
                {
                    if(permutation[best_ts_permutation_idx][i] != -1) matched_row_index_sec1.push_back(i);
                }
            }else{
                for(int i=0; i<permutation[best_ts_permutation_idx].size(); i++)
                {
                    if(permutation[best_ts_permutation_idx][i] != -1) matched_row_index_sec1.push_back(permutation[best_ts_permutation_idx][i]);
                }
            }
            
            // matched col index in sec1 (needed in learning stage)
            int i=_best_back_pointers.size()-1, j=_best_back_pointers[0].size()-1;
            while(i!=0 & j!=0)
            {
                if(std::strcmp(_best_back_pointers[i][j].c_str(), "addxy")==0)
                {
                    if(derSec1.row <= derSec2.row)
                    {
                        matched_col_index_sec1.push_back(i-1);matched_col_index_sec1.push_back(i);
                        matched_col_index_sec2.push_back(j-1);matched_col_index_sec2.push_back(j);
                    }else{
                        matched_col_index_sec1.push_back(j-1);matched_col_index_sec1.push_back(j);
                        matched_col_index_sec2.push_back(i-1);matched_col_index_sec2.push_back(i);
                    }
                    
                    i--; j--;
                }else if(std::strcmp(_best_back_pointers[i][j].c_str(), "skipx")==0){
                    i--;
                }else if(std::strcmp(_best_back_pointers[i][j].c_str(), "skipy")==0){
                    j--;
                }
            }
        }
        
        std::sort(matched_col_index_sec1.begin(), matched_col_index_sec1.end());
        matched_col_index_sec1.erase(std::unique(matched_col_index_sec1.begin(), matched_col_index_sec1.end()), matched_col_index_sec1.end());
        
        std::sort(matched_col_index_sec2.begin(), matched_col_index_sec2.end());
        matched_col_index_sec2.erase(std::unique(matched_col_index_sec2.begin(), matched_col_index_sec2.end()), matched_col_index_sec2.end());
        
        return best_ts/100;
    }
    
    float similarityMeasure::subString(std::vector< std::string> row1, std::vector< std::string> row2, const char* option)
    {
        int length1 = row1.size();
        int length2 = row2.size();
        int shift_times = std::abs<int>(length1-length2) + 1;
    
        //row1 length < row 2 length (swap if necessary))
        if(length1 > length2) row1.swap(row2);
        int normalization = row2.size();
        
        //sub-string search: row1 vs row2[shift:shift+(length1-1))]]
        float f_shift[shift_times];
        float f = 0;
        for(int i=0; i<shift_times; i++)
        {
            int count = 0;
            for(int j=0; j<row1.size(); j++)
            {
                int index1 = j;
                int index2 = i+j;
                if(std::strcmp(row1[index1].c_str(), row2[index2].c_str())==0) count = count + 1;
            }
            f_shift[i] = (float)100/normalization*count;
//             std::cout << f_shift[i] << " ";
            
            if(f_shift[i] > f) f = f_shift[i];
        }
        std::cout << f;
        if(std::strcmp(option, "spatial")==0)
        {
            if(f >= _spatial_threshold)
            {
                f=100;
                std::cout << "->" << f << " ";
            }else{
                std::cout << " ";
            }
        }else if(std::strcmp(option, "temporal")==0)
        {
            /*if(f >= _temporal_threshold)
            {
                f=100;
                std::cout << "->" << f << " ";
            }else*/
            {
                std::cout << " ";
            }
        }
        
        return f;
    }
    
    void similarityMeasure::permute(std::vector< std::vector< int > > match_map, std::vector< std::vector< int > >& permutations, std::vector<int> temp_permutation)
    {
        // check for update of match_map
        if(!temp_permutation.empty())
        {
            // find which rows has already been assigned
            int count = 0;
            for(int i=0; i<temp_permutation.size(); i++)
            {
                if(temp_permutation[i] != -1)
                {
                    // update match_map 
                    // 1. entries equal to temp_permutation[i] set to -1)
                    // 2. row_i of match_map set to -1
                    for(int p=0; p<match_map.size(); p++)
                    { 
                        for(int q=0; q<match_map[p].size(); q++)
                        {
                            // check whether several rows match to only one row
                            if(match_map[p].size()==1 & match_map[p][q] == temp_permutation[i] & p!=i)
                            {
                                std::vector<int> temp_permutation_special = temp_permutation;
                                temp_permutation_special[p] = temp_permutation[i];
                                temp_permutation_special[i] = -1;
                                permute(match_map, permutations, temp_permutation_special);
//                                 return;
                            }
                            if(match_map[p][q] == temp_permutation[i] | p==i)
                            {
                                match_map[p][q] = -1;
                            }
                        }
                    }
                    count = count + 1;
                }
            }
            // check terminal condition
            if(count == temp_permutation.size()) 
            {
                permutations.push_back(temp_permutation);
                return;
            }
        }
        
        // get # of matches for each row
        std::vector<int> match_num;
        for(int i=0; i<match_map.size(); i++)
        {
            match_num.push_back(0);;
            for(int j=0; j<match_map[i].size(); j++)
            {
                if(match_map[i][j] != -1) match_num[i] = match_num[i] + 1;
            }
        }
        
        // find rows with least # of matches
        int min_match_num = 1000;
        for(int i=0; i<match_map.size(); i++)
        {
            if( match_num[i] < min_match_num & match_num[i] > 0 ) min_match_num = match_num[i];
        }
        // check for terminal condition
        if( min_match_num == 1000)
        {
            if(!temp_permutation.empty()) permutations.push_back(temp_permutation);
            return;
        }
        int min_match_row; // index of the 1st row that has the least # of matches
        for(int i=0; i<match_map.size(); i++)
        {
            if( match_num[i] == min_match_num)
            {
                min_match_row = i;
                break;
            }
        }
        
        // start permutations from the min_match_row
        for(int i=0; i<match_map[min_match_row].size(); i++)
        {
            if(temp_permutation.empty()) temp_permutation = std::vector< int >(match_map.size(), -1);
            
            if( match_map[min_match_row][i] != -1) 
            {
                temp_permutation[min_match_row] = match_map[min_match_row][i];
                permute(match_map, permutations, temp_permutation);
            }
            
        }
    }
    
    void similarityMeasure::lss(int lss_length, std::vector< std::vector< std::string > >& b)
    {
        std::vector<std::vector<int>> c;
       
        // initialization
        for(int i=0; i<=_ts.size(); i++)
        {
            std::vector<int> c_row(_ts[0].size()+1);
            c.push_back(c_row);
            
            std::vector<std::string> b_row(_ts[0].size()+1);
            b.push_back(b_row);
        }
        for(int i=0; i<=_ts.size(); i++)
        {
            c[i][0]=0;
            b[i][0]=std::string("skipx");
        }
        for(int j=0; j<=_ts[0].size(); j++)
        {
            c[0][j]=0;
            b[0][j]=std::string("skipy");
        }
        
        // find the max of each row of _ts
        std::vector<float> ts_row_max;
        for(int i=0; i<_ts.size(); i++)
        {
            float row_max = 0;
            for(int j=0; j<_ts[i].size(); j++)
            {
                if(_ts[i][j] > row_max) row_max = _ts[i][j];
            }
            ts_row_max.push_back(row_max);
        }
        
        // count for length of lss, and store the back pointers in b
        for(int i=1; i<=_ts.size(); i++)
        {
            for(int j=1; j<=_ts[0].size(); j++)
            {
//                 if(_ts[i-1][j-1]>60 & ts_row_max[i-1]>0)
                if(_ts[i-1][j-1]==ts_row_max[i-1] & ts_row_max[i-1]>0)
                {
                    c[i][j]=c[i-1][j-1]+1;
                    b[i][j]=std::string("addxy");
                }else if(c[i-1][j] >= c[i][j-1]){
                    c[i][j]=c[i-1][j];
                    b[i][j]=std::string("skipx");
                }else{
                    c[i][j]=c[i][j-1];
                    b[i][j]=std::string("skipy");
                }
            }
        }
        
        lss_length = c[_ts.size()][_ts[0].size()];
        std::cout << "longest similar subsequence length = " << lss_length << std::endl;
    }

}