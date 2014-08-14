/**
 * \file        mainGraph.cpp
 * \author      Zhen Zeng (zengzhen@umich.edu) 
 */

#include "mainGraph.h"
#include <fstream>

namespace TableObject{
    
    mainGraph::mainGraph(int start_index)
    {
        graph newGraph;
        newGraph.video_index = start_index;
        _graph.push_back(newGraph);
    }

    void mainGraph::addInitialRelationalGraph(int relation)
    {
        _pre_graph.push_back(relation);
        _graph[0].relational_graph.push_back(relation);
    }

    
    void mainGraph::addRelationalGraph(int relation)
    {
        _cur_graph.push_back(relation);
    }

    bool mainGraph::compareRelationGraph(int frame_index)
    {
        bool change = false;
        // print out pre and cur relational graph
        std::cout << "pre relational scene graph: ";
        for(int i=0; i<_pre_graph.size(); i++)
        {
            std::cout << _pre_graph[i] << " ";
        }
        std::cout << "\ncur relational scene graph: ";
        for(int i=0; i<_cur_graph.size(); i++)
        {
            std::cout << _cur_graph[i] << " ";
        }
        std::cout << std::endl;
        
        //compare previous and current frame relational graph
        for(int i=0; i<_cur_graph.size(); i++)
        {
            if(_cur_graph[i]!=_pre_graph[i])
            {
                graph newGraph;
                newGraph.video_index = frame_index;
                newGraph.relational_graph = _cur_graph;
                _graph.push_back(newGraph);
                change = true;
                break;
            }
        }
        
        //update previous frame graph to be curreent graph
        _pre_graph = _cur_graph;
        _cur_graph.clear();
        
        return change;
    }

    std::vector<graph> mainGraph::getMainGraph()
    {
        return _graph;
    }
    
    void mainGraph::displayMainGraph()
    {
        for(int i=0; i<_graph.size(); i++)
        {
            std::cout << "\nframe " << _graph[i].video_index << ": ";
            for(int j=0; j<_graph[i].relational_graph.size(); j++)
            {
                std::cout << _graph[i].relational_graph[j] << " ";
            }
        }
        std::cout << std::endl;
    }

//     void mainGraph::recordMainGraph(std::string file_name)
//     {
//         std::ofstream mainGraph_file(file_name.c_str());
//         for(int i=0; i<_graph.size(); i++)
//         {
//             for(int j=0; j<_graph[i].relational_graph.size(); j++)
//             {
//                 mainGraph_file << _graph[i].relational_graph[j] << " ";
//             }
//             mainGraph_file << std::endl;
//         }
//         mainGraph_file.close();
//         std::cout << "main graph saved at " << file_name << std::endl;
//     }
    
        /**
     *  mainGraph_file.txt:
     *       row: relation between object i and object j from frame N1 to frame N2
     *       col: different object pairs
     */
    
    void mainGraph::recordMainGraph(std::string file_name)
    {
        std::ofstream mainGraph_file(file_name.c_str());
        for(int i=0; i<_graph[0].relational_graph.size(); i++)
        {
            for(int j=0; j<_graph.size(); j++)
            {
                mainGraph_file << _graph[j].relational_graph[i] << " ";
            }
            mainGraph_file << std::endl;
        }
        mainGraph_file.close();
        std::cout << "main graph saved at " << file_name << std::endl;
    }

}