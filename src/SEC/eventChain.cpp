/**
 * \file        eventChain.cpp
 * \author      Zhen Zeng (zengzhen@umich.edu) 
 */

#include "eventChain.h"
#include <fstream>
#include <string.h>

namespace TableObject{
    
    eventChain::eventChain()
    {
        _original_sec.row = 0;
        _derivative_sec.row = 0;
        _compressed_sec.row = 0;
    }

    eventChain::eventChain(std::string& mainGraph_file)
    {
        _original_sec.row = 0;
        _derivative_sec.row = 0;
        _compressed_sec.row = 0;
        
        std::string line;
        std::ifstream mainGraph(mainGraph_file.c_str());
        if (mainGraph.is_open())
        {
//             int count=0;
            while ( getline (mainGraph,line,'\n') )
            {
                std::istringstream str(line);
                std::vector<std::string> sec_row;
                std::string relation;
                
                while ( getline (str,relation,' ') )
                {
                    std::string each_relation(relation);
                    sec_row.push_back(each_relation);
//                     std::cout << each_relation.c_str() << " ";
                }
                _original_sec.event_chain.push_back(sec_row);
                _original_sec.row = _original_sec.row + 1;
                _original_sec.cols.push_back(sec_row.size());
//                 std::cout << std::endl;
//                 if(count>=1)
//                 {
//                     std::cout << "test: ";
//                     for(int i=0; i<sec_row.size(); i++)
//                     {
//                         std::string test = sec_row[i];
//                         std::cout << test.c_str() <<  " ";
//                     }
//                     std::cout << std::endl;
//                 }
//                 count = count + 1;
            }
            mainGraph.close();
        }
        
        // erase rows where nothing happens in semantic event chain
        for(int i=_original_sec.row-1; i>=0; i--)
        {
            bool nothing_happens = true;
            for(int j=1; j<_original_sec.cols[i]; j++)
            {
                char str[80];
                std::strcpy(str, _original_sec.event_chain[i][j-1].c_str());
                std::strcat(str, _original_sec.event_chain[i][j].c_str());
                
                nothing_happens = (std::strcmp( _original_sec.event_chain[i][j-1].c_str() , _original_sec.event_chain[i][j].c_str() ) == 0) & nothing_happens;
            }
            if(nothing_happens)
            {
                _original_sec.row = _original_sec.row - 1;
                _original_sec.cols.erase(_original_sec.cols.begin()+i);
                _original_sec.event_chain.erase(_original_sec.event_chain.begin()+i);
            }
        }
        
    }
    
    void eventChain::takeDerivative()
    {
        for(int i=0; i<_original_sec.row; i++)
        {
            std::vector<std::string> sec_row;
//             bool nothing_happens = true;
            for(int j=1; j<_original_sec.cols[i]; j++)
            {
                char str[80];
                std::strcpy(str, _original_sec.event_chain[i][j-1].c_str());
                std::strcat(str, _original_sec.event_chain[i][j].c_str());
                sec_row.push_back(std::string(str));
                
//                 nothing_happens = (std::strcmp( _original_sec.event_chain[i][j-1].c_str() , _original_sec.event_chain[i][j].c_str() ) == 0) & nothing_happens;
            }
//             if(!nothing_happens)
            {
                _derivative_sec.event_chain.push_back(sec_row);
                _derivative_sec.row = _derivative_sec.row + 1;
                _derivative_sec.cols.push_back(_original_sec.cols[i]-1);
            }
        }
    }

    void eventChain::compress()
    {
        char term1[] = "00";
        char term2[] = "22";
        
        for(int i=0; i<_derivative_sec.row; i++)
        {
            std::vector<std::string> sec_row;
            
            for(int j=0; j<_derivative_sec.cols[i]; j++)
            {
                bool remove1 = false;
                bool remove2 = false;
                remove1 = std::strcmp( _derivative_sec.event_chain[i][j].c_str() , term1 ) == 0;
                remove2 = std::strcmp( _derivative_sec.event_chain[i][j].c_str() , term2 ) == 0;
                
                if(!remove1 & !remove2) sec_row.push_back(_derivative_sec.event_chain[i][j]);
            }
            _compressed_sec.event_chain.push_back(sec_row);
            _compressed_sec.row = _compressed_sec.row + 1;
            _compressed_sec.cols.push_back(sec_row.size());
        }
    }
    
    void eventChain::getOriginalSec(sec& original_sec)
    {
        original_sec = _original_sec;
    }
    
    void eventChain::getDerivativeSec(sec& derivative_sec)
    {
        derivative_sec = _derivative_sec;
    }

    void eventChain::getCompressedSec(sec& compressed_sec)
    {
        compressed_sec = _compressed_sec;
    }
    
    void eventChain::setOriginalSec(sec& sec)
    {
        _original_sec = sec;
    }
    
    void eventChain::display(const char* sec_type)
    {
        if(std::strcmp(sec_type, "original") == 0)
        {
            std::cout << "original ";
            _original_sec.display();
        }else if(std::strcmp(sec_type, "derivative") == 0)
        {
            std::cout << "derivative ";
            _derivative_sec.display();
        }else if(std::strcmp(sec_type, "compressed") == 0)
        {
            std::cout << "compressed ";
            _compressed_sec.display();
        }else if(std::strcmp(sec_type, "all") == 0)
        {
            std::cout << "original "; _original_sec.display();
            std::cout << "derivative "; _derivative_sec.display();
            std::cout << "compressed "; _compressed_sec.display();
        }
            
    }


}