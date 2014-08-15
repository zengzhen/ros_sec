/**
 * \file        similarityMeasure.h
 * \author      Zhen Zeng (zengzhen@umich.edu)
 * \brief       similarity measure of two semantic evant chains
 */

#ifndef SIMILARITYMEASURE_H
#define SIMILARITYMEASURE_H

#include <Eigen/Dense>

#include "ros_sec/SEC/eventChain.h"
#include <string.h>

namespace TableObject{
    
    class similarityMeasure{
    public:
        /** \brief empty Constructor
        */
        similarityMeasure(){};
        
        /** \brief Constructor
         *  \param[in] sec1 1st semantic event chain to be compared
         *  \param[in] sec2 2nd semantic event chain to be compared
        */
        similarityMeasure(const eventChain& sec1, const eventChain& sec2);
        
        /** \brief set threshold above which one would consider two spatial similarity values as equal
         *  \param[in] spatial_threshold threshold (eg. 0.8f)
        */
        void setSpatialThreshold(float spatial_threshold);
        
        /** \brief set threshold above which one would consider two temporal similarity values as equal
         *  \param[in] temporal_threshold threshold (eg. 0.8f)
        */
        void setTemporalThreshold(float temporal_threshold);

        /** \brief spatial similarity between two semantic event chains given while construction
        */
        float spatialSimilarity();
        
        /** \brief temporal similarity between two semantic event chains given while construction
         *  \param[out] add_row_index_sec2 while learning, if there are new rows occurred in sec2, store the index for the rows to be added to the model
         *  \param[out] matched_row_index_sec1 matched row index in sec1
         *  \param[out] matched_col_index_sec1 matched col index in sec1
         *  \param[out] matched_col_index_sec2 matched col index in sec2
        */
        float temporalSimilarity(std::vector<int>& add_row_index_sec2, 
                                 std::vector<int>& matched_row_index_sec1, 
                                 std::vector<int>& matched_col_index_sec1,
                                 std::vector<int>& matched_col_index_sec2);
        
    private:
        eventChain _sec1;
        eventChain _sec2;
        float _spatial_threshold;
        float _temporal_threshold;
        float _spatial_equal_threshold;
        
        std::vector<std::vector<float>> _ss;
        std::vector<std::vector<float>> _ts;
        std::vector<std::vector<std::string>> _best_back_pointers;
        
        /** \brief calculate max(similarity(shifted version of short string array, another long string array) )
         *  \param[in] row1 1st string array
         *  \param[in] row2 2nd string array
         *  \param[in] option spatial or temporal similarity
        */
        float subString(std::vector<std::string> row1, std::vector<std::string> row2, const char* option);
        
        /** \brief recursive function to find all possible permutations of rows of compressed sec1 to match compressed_sec2
         *  \param[in] match_map row_i of compressed_sec1 is the same with row_match_map[i][j] of compressed_sec2
         *  \param[out] permutations all possible permutations where each row represents one possible permutation
         *                           in ith permutation, row_j of compressed_sec1 is assigned to row_permutations[i][j] of compressed_sec2
        */
        void permute(std::vector<std::vector<int>> match_map, std::vector<std::vector<int>>& permutations, std::vector<int> temp_permutation);
        
        /** \brief find longest similar subsequence using dynamic programming
         *  \param[out] lss_length the length of the longest similar subsequence
         *  \param[out] b back pointers for extracting longest similar subsequence
        */
        void lss(int lss_length, std::vector<std::vector<std::string>>& b);
    };
}

#endif
    