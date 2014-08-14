/**
 * \file        testEventChain.cpp
 * \author      Zhen Zeng (zengzhen@umich.edu)
 * \brief       test event chain class
 */

#include "SEC/eventChain.h"

int main(int argc, char** argv)
{
    if(argc!=2)
    {
        std::cerr << "Usage: ./testEventChain DEMO_NAME" << std::endl;
        exit(1);
    }
    
    std::string demo_name=argv[1];
    std::string mainGraph_file;
    mainGraph_file = "../../../result/" + demo_name + "/mainGraph.txt";
    
    TableObject::eventChain event_chain(mainGraph_file);
    event_chain.display("original");
    event_chain.takeDerivative();
    event_chain.display("derivative");
    event_chain.compress();
    event_chain.display("compressed");
    return 0;
}