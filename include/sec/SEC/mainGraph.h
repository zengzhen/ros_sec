/**
 * \file        mainGraph.h
 * \author      Zhen Zeng (zengzhen@umich.edu)
 * \brief       main graph of manipulation action
 */

#ifndef MAINGRAPH_H
#define MAINGRAPH_H

#include "typeDef.h"

namespace TableObject{
    
    class mainGraph {
    public:
        /** \brief Constructor
         *  \param[in] start_index index of starting frame of processed video
        */
        mainGraph(int start_index);
        
        /** Add initial relational graph for the 1st frame
         *  \param[in] relation digit representing the relation (0: untouch, 2: touch)
         */
        void addInitialRelationalGraph(int relation);
        
        /** Add relational graph for frames after the 1st frame
         *  \param[in] relation digit representing the relation (0: untouch, 2: touch)
         */
        void addRelationalGraph(int relation);
        
        /** Compare relational graphs between frame_index and frame_index-1
         *  \param[in] frame_index the index of the frame to be compared
         */
        bool compareRelationGraph(int frame_index);
        
        /** Accessor main graph
         */
        std::vector<graph> getMainGraph();
        
        /** Display main graph in terminal
        */
        void displayMainGraph();
        
        /** Record main graph in file
         * \param[in] file_name file to be written to
        */
        void recordMainGraph(std::string file_name);
        
    private:
        
        std::vector<graph> _graph; 
        std::vector<int> _cur_graph; //to store relational graph of current frame for comparison
        std::vector<int> _pre_graph; //to store relational graph of previous frame for comparison
        
    };
}

#endif // MAINGRAPH_H