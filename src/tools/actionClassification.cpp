/**
 * \file        actionClassification.cpp
 * \author      Zhen Zeng (zengzhen@umich.edu)
 * \brief       learn a actoin classifier
 */

#include "SEC/similarityMeasure.h"

int main(int argc, char** argv)
{
    if(argc<3 | argc >4)
    {
        std::cerr << "Usage: ./actionClassification DATASET_PATH CLASS_NAME (opt)threshold(60)" << std::endl;
        std::cerr << "class: push, pick_up, stack, stack_unstack" << std::endl;
        exit(1);
    }
    
    float threshold = 60;
    if(argc==4) threshold = std::stof(argv[3]);
    
    // construct path for every training instance
    std::vector<char*> training_set;
    
    if(std::strcmp(argv[2], "stack_unstack")==0)
    {
        for(int i=0; i<5; i++)
        {
            training_set.push_back(new char[500]);
        }
        // stack_unstack training set
        std::snprintf(training_set[0], 500, "%s/stack_unstack/demo8_4", argv[1]);
        std::snprintf(training_set[1], 500, "%s/stack_unstack/demo8_3", argv[1]);
        std::snprintf(training_set[2], 500, "%s/stack_unstack/demo8_2", argv[1]);
        std::snprintf(training_set[3], 500, "%s/stack_unstack/demo8_1", argv[1]);
//         std::snprintf(training_set[4], 500, "%s/stack_unstack/demo11_2", argv[1]);
    }else if(std::strcmp(argv[2], "push")==0){
        for(int i=0; i<4; i++)
        {
            training_set.push_back(new char[500]);
        }
        // push training set
        std::snprintf(training_set[0], 500, "%s/push/demo1_1", argv[1]);
        std::snprintf(training_set[1], 500, "%s/push/demo1_2", argv[1]);
        std::snprintf(training_set[2], 500, "%s/push/demo2_1", argv[1]);
        std::snprintf(training_set[3], 500, "%s/push/demo2_2", argv[1]);
    }else if(std::strcmp(argv[2], "pick_up")==0){
        for(int i=0; i<4; i++)
        {
            training_set.push_back(new char[500]);
        }
        std::snprintf(training_set[0], 500, "%s/pick_up/demo4_1", argv[1]);
        std::snprintf(training_set[1], 500, "%s/pick_up/demo5_2", argv[1]);
        std::snprintf(training_set[2], 500, "%s/pick_up/demo5_3", argv[1]);
        std::snprintf(training_set[3], 500, "%s/pick_up/demo5_4", argv[1]);
    }else if(std::strcmp(argv[2], "stack")==0){
        for(int i=0; i<4; i++)
        {
            training_set.push_back(new char[500]);
        }
        std::snprintf(training_set[0], 500, "%s/stack/demo7_1", argv[1]);
        std::snprintf(training_set[1], 500, "%s/stack/demo8_1", argv[1]);
        std::snprintf(training_set[2], 500, "%s/stack/demo8_2", argv[1]);
        std::snprintf(training_set[3], 500, "%s/stack/demo8_3", argv[1]);
    }
    
    // construct weight vectors
    std::vector<float> weight_row;    
    std::vector<float> weight_col;    
    float delta = (float)1/training_set.size();
    
    // start learning
    TableObject::eventChain learned_event_chain;
    for(int i=0; i<training_set.size()-1; i++)
    {
        std::string mainGraph_file_1;
        std::string mainGraph_file_2;
        mainGraph_file_1 = std::string(training_set[i]) + "/mainGraph.txt";
        mainGraph_file_2 = std::string(training_set[i+1]) + "/mainGraph.txt";
        
        TableObject::eventChain event_chain1(mainGraph_file_1);
        event_chain1.takeDerivative();
        event_chain1.compress();
        
        TableObject::eventChain event_chain2(mainGraph_file_2);
        event_chain2.takeDerivative();
        event_chain2.compress();
        
        if(i==0) 
        {
            learned_event_chain = event_chain1;
            sec original_sec;
            event_chain1.getOriginalSec(original_sec);
            for(int p=0; p<original_sec.row; p++)
            {
                weight_row.push_back(1.5*delta); //initial weights
            }
            for(int p=0; p<original_sec.cols[0]; p++)
            {
                weight_col.push_back(1.5*delta); //initial weights
            }
        }
        
        TableObject::similarityMeasure sm(event_chain1, event_chain2);
        std::vector<int> add_row_index_sec2;
        std::vector<int> matched_row_index_sec1;
        std::vector<int> matched_col_index_sec1;
        std::vector<int> matched_col_index_sec2;
        sm.setSpatialThreshold(0.6f);
        sm.spatialSimilarity();
        sm.setTemporalThreshold(0.6f);
        float ts = sm.temporalSimilarity(add_row_index_sec2, matched_row_index_sec1, matched_col_index_sec1, matched_col_index_sec2);
        
        std::cout << "add_row_index_sec2:";
        for(int p=0 ; p<add_row_index_sec2.size(); p++) std::cout << add_row_index_sec2[p] << " ";
        std::cout << std::endl;
        
        std::cout << "matched_row_index_sec1:";
        for(int p=0 ; p<matched_row_index_sec1.size(); p++) std::cout << matched_row_index_sec1[p] << " ";
        std::cout << std::endl;
        
        std::cout << "matched_col_index_sec1:";
        for(int p=0 ; p<matched_col_index_sec1.size(); p++) std::cout << matched_col_index_sec1[p] << " ";
        std::cout << std::endl;
        
        // update learned event chain
        if(ts > threshold/100)
        {
            // add new unobserved rows
            // !! not clear how to add based on the original paper
//                 sec original_sec;
//                 event_chain1.getOriginalSec(original_sec);
//                 for(int i=0; i<add_row_index_sec2.size(); i++)
//                 {
//                     original_sec.row = original_sec.row + 1;
//                     original_sec.cols.push_back(original_sec.cols[0]);
//                     std::vector<std::string> add_sec_row;
//                     
//                 }

            // increase weights for matched cols and rows
            for(int j=0; j<matched_row_index_sec1.size(); j++)
            {
                weight_row[matched_row_index_sec1[j]] = weight_row[matched_row_index_sec1[j]] + delta;
            }
            for(int j=0; j<matched_col_index_sec1.size(); j++)
            {
                weight_col[matched_col_index_sec1[j]] = weight_col[matched_col_index_sec1[j]] + delta;
            }
        }
        
    }
    
    // diplay weight vectors
    learned_event_chain.display("original");
    std::cout << "row weights: ";
    std::vector<int> remove_row;
    std::vector<int> remove_col;
    for(int i=0; i<weight_row.size(); i++)
    {
        std::cout << weight_row[i] << " ";
        if(weight_row[i]<0.5f) remove_row.push_back(i);
    }
    std::cout << std::endl;
    std::cout << "col weights: ";
    for(int i=0; i<weight_col.size(); i++)
    {
        std::cout << weight_col[i] << " ";
        if(weight_col[i]<0.5f) remove_col.push_back(i);
    }
    std::cout << std::endl;
    
    // threshold learned sec weights
    sec original_sec;
    learned_event_chain.getOriginalSec(original_sec);
    for(int i=remove_row.size()-1;i>=0; i--)
    {
        original_sec.row = original_sec.row - 1;
        original_sec.event_chain.erase(original_sec.event_chain.begin()+remove_row[i]);
    }
    for(int i=remove_col.size()-1;i>=0; i--)
    {
        for(int j=0;j<original_sec.row; j++)
        {
            original_sec.cols[j] = original_sec.cols[j] - 1;
            original_sec.event_chain[j].erase(original_sec.event_chain[j].begin() + remove_col[i]);
        }
    }
    learned_event_chain.setOriginalSec(original_sec);
    
    // display thresholded event chain
    learned_event_chain.display("original");
    
}