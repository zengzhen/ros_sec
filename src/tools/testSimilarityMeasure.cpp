/**
 * \file        testSimilarityMeasure.cpp
 * \author      Zhen Zeng (zengzhen@umich.edu)
 * \brief       test similarity measure
 */

#include "SEC/similarityMeasure.h"

int main(int argc, char** argv)
{
    if(argc!=3)
    {
        std::cerr << "Usage: ./testSimilarityMeasure DEMO_NAME1 DEMO_NAME2" << std::endl;
        exit(1);
    }
    
    std::string demo_name1=argv[1];
    std::string demo_name2=argv[2];
    std::string mainGraph_file_1;
    std::string mainGraph_file_2;
    mainGraph_file_1 = "../../../result/" + demo_name1 + "/mainGraph.txt";
    mainGraph_file_2 = "../../../result/" + demo_name2 + "/mainGraph.txt";
    
    
    TableObject::eventChain event_chain1(mainGraph_file_1);
    event_chain1.takeDerivative();
    event_chain1.compress();
    
    TableObject::eventChain event_chain2(mainGraph_file_2);
    event_chain2.takeDerivative();
    event_chain2.compress();

    std::cout << "sec1: " << std::endl;
    event_chain1.display("compressed");
//     event_chain1.display("derivative");
    std::cout << std::endl;

    std::cout << "sec2: " << std::endl;
    event_chain2.display("compressed");
//      event_chain2.display("derivative");
    std::cout << std::endl;
    
    TableObject::similarityMeasure sm(event_chain1, event_chain2);
    std::vector<int> add_row_index_sec2;
    std::vector<int> matched_row_index_sec1;
    std::vector<int> matched_col_index_sec1;
    std::vector<int> matched_col_index_sec2;
    sm.setSpatialThreshold(0.8f);
    sm.spatialSimilarity();
    sm.setTemporalThreshold(0.8f);
    sm.temporalSimilarity(add_row_index_sec2, matched_row_index_sec1, matched_col_index_sec1, matched_col_index_sec2);
    
    // display results
    std::cout << "add_row_index_sec2:";
    for(int i=0 ; i<add_row_index_sec2.size(); i++) std::cout << add_row_index_sec2[i] << " ";
    std::cout << std::endl;
    
    std::cout << "matched_row_index_sec1:";
    for(int i=0 ; i<matched_row_index_sec1.size(); i++) std::cout << matched_row_index_sec1[i] << " ";
    std::cout << std::endl;
    
    std::cout << "matched_col_index_sec1:";
    for(int i=0 ; i<matched_col_index_sec1.size(); i++) std::cout << matched_col_index_sec1[i] << " ";
    std::cout << std::endl;
    
    return 0;
}