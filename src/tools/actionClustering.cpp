/**
 * \file        actionClustering.cpp
 * \author      Zhen Zeng (zengzhen@umich.edu)
 * \brief       measure the similarities between every pair of action instances in the dataset
 */

#include "SEC/similarityMeasure.h"

int main(int argc, char** argv)
{
    if(argc<2 | argc >3)
    {
        std::cerr << "Usage: ./actionClustering DATASET_PATH (opt)threshold(60)" << std::endl;
        exit(1);
    }
    
    float threshold = 60;
    if(argc==3) threshold = std::stof(argv[2]);
    
    // construct path for every training instance
    std::string data_path=argv[1];
    std::vector<char*> training_set;
    for(int i=0; i<16; i++)
    {
        training_set.push_back(new char[500]);
    }
    
    // push training set
//     char push1[50], push2[50], push3[50], push4[50];
    std::snprintf(training_set[0], 500, "%s/push/demo1_1", argv[1]);
    std::snprintf(training_set[1], 500, "%s/push/demo1_2", argv[1]);
    std::snprintf(training_set[2], 500, "%s/push/demo2_1", argv[1]);
    std::snprintf(training_set[3], 500, "%s/push/demo2_2", argv[1]);
    // pick_up training set
//     char pick_up1[50], pick_up2[50], pick_up3[50], pick_up4[50];
    std::snprintf(training_set[4], 500, "%s/pick_up/demo4_1", argv[1]);
    std::snprintf(training_set[5], 500, "%s/pick_up/demo5_2", argv[1]);
    std::snprintf(training_set[6], 500, "%s/pick_up/demo5_3", argv[1]);
    std::snprintf(training_set[7], 500, "%s/pick_up/demo5_4", argv[1]);
    // stack training set
    std::snprintf(training_set[8], 500, "%s/stack/demo7_1", argv[1]);
    std::snprintf(training_set[9], 500, "%s/stack/demo8_1", argv[1]);
    std::snprintf(training_set[10], 500, "%s/stack/demo8_2", argv[1]);
    std::snprintf(training_set[11], 500, "%s/stack/demo8_3", argv[1]);
    // stack_unstack training set
    std::snprintf(training_set[12], 500, "%s/stack_unstack/demo8_1", argv[1]);
    std::snprintf(training_set[13], 500, "%s/stack_unstack/demo8_2", argv[1]);
    std::snprintf(training_set[14], 500, "%s/stack_unstack/demo8_3", argv[1]);
    std::snprintf(training_set[15], 500, "%s/stack_unstack/demo8_4", argv[1]);
    
    // construct similarity matrix
    std::vector<std::vector<float>> similarity_matrix;
    for(int i=0; i<16; i++)
    {
        std::vector<float> similarity_row;
        for(int j=0; j<16; j++)
        {
            similarity_row.push_back(-1);
        }
        similarity_matrix.push_back(similarity_row);
    }
    
    // measure pairwise similarity
    for(int i=0; i<16; i++)
    {
        for(int j=i+1; j<16; j++)
        {
            std::string mainGraph_file_1;
            std::string mainGraph_file_2;
            mainGraph_file_1 = std::string(training_set[i]) + "/mainGraph.txt";
            mainGraph_file_2 = std::string(training_set[j]) + "/mainGraph.txt";
            
            TableObject::eventChain event_chain1(mainGraph_file_1);
            event_chain1.takeDerivative();
            event_chain1.compress();
            
            TableObject::eventChain event_chain2(mainGraph_file_2);
            event_chain2.takeDerivative();
            event_chain2.compress();
            
            TableObject::similarityMeasure sm(event_chain1, event_chain2);
            std::vector<int> add_row_index_sec2;
            std::vector<int> matched_row_index_sec1;
            std::vector<int> matched_col_index_sec1;
            std::vector<int> matched_col_index_sec2;
            sm.setSpatialThreshold(0.8f);
            sm.spatialSimilarity();
            sm.setTemporalThreshold(0.8f);
            float ts = sm.temporalSimilarity(add_row_index_sec2, matched_row_index_sec1, matched_col_index_sec1, matched_col_index_sec2);
            
            similarity_matrix[i][j] = ts;
            similarity_matrix[j][i] = ts;
        }
        similarity_matrix[i][i] = 1;
    }
    
    // diplay simlarity matrix
     for(int i=0; i<16; i++)
    {
        for(int j=0; j<16; j++)
        {
            std::cout << similarity_matrix[i][j] << " ";
        }
        std::cout << std::endl;
    }
    
    
    
}