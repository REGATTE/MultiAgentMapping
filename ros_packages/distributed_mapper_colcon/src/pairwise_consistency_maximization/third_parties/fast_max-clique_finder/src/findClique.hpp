#ifndef _FINDCLIQUE_H_INCLUDED_
#define _FINDCLIQUE_H_INCLUDED_

#include "graphIO.hpp"
#include <cstddef>
#include <iostream>
#include <sys/time.h>
#include <unistd.h>
#include <stdlib.h>
#include <numeric>
#include <algorithm>
using namespace std;

#ifdef _DEBUG
int DEBUG=1;
#endif

namespace FMC {
    //Function Definitions
    bool fexists(const char *filename);
    double wtime();
    void usage(char *argv0);
    int getDegree(vector<int>* ptrVtx, int idx);
    void print_max_clique(vector<int>& max_clique_data);

    int maxClique( CGraphIO& gio, int l_bound, vector<int>& max_clique_data );
    void maxCliqueHelper( CGraphIO& gio, vector<int>* U, int sizeOfClique, int& maxClq, vector<int>& max_clique_data_inter );

    int maxCliqueHeu( CGraphIO& gio );
    int maxCliqueHeu(CGraphIO& gio, vector<int>& max_clique_data);
    void maxCliqueHelperHeu( CGraphIO& gio, vector<int>* U, int sizeOfClique, int& maxClq, vector<int>& max_clique_data_inter );

}
#endif // _FINDCLIQUE_H_INCLUDED_
