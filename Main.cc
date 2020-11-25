/*!
 * \author Roberto Asin-Acha - rasin@inf.udec.cl
 *
 * @section LICENSE
 *
 * MiniSat,  Copyright (c) 2003-2006, Niklas Een, Niklas Sorensson
 *           Copyright (c) 2007-2010, Niklas Sorensson
 * Open-WBO, Copyright (c) 2013-2017, Ruben Martins, Vasco Manquinho, Ines Lynce
 * MAPFtoMaxSAT, Copyright (c) 2020- , Roberto Asin Acha
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 */

#include "utils/Options.h"
#include "utils/ParseUtils.h"
#include "utils/System.h"
#include <errno.h>
#include <signal.h>
#include <zlib.h>

#include <fstream>
#include <iostream>
#include <map>
#include <stdlib.h>
#include <string>
#include <vector>
#include "MAPFEncoder/MAPFtoMaxSAT.hh"

#ifdef SIMP
#include "simp/SimpSolver.h"
#else
#include "core/Solver.h"
#endif

#include "MaxSAT.h"
#include "MaxTypes.h"

// Algorithms
#include "algorithms/Alg_LinearSU.h"
#include "algorithms/Alg_PartMSU3.h"
#include "algorithms/Alg_MSU3.h"
#include "algorithms/Alg_OLL.h"

#define VER1_(x) #x
#define VER_(x) VER1_(x)
#define SATVER VER_(SOLVERNAME)
#define VER VER_(VERSION)

using NSPACE::cpuTime;
using NSPACE::OutOfMemoryException;
using NSPACE::IntOption;
using NSPACE::BoolOption;
using NSPACE::StringOption;
using NSPACE::IntRange;
using NSPACE::parseOptions;
using namespace openwbo;

//=================================================================================================

static MaxSAT *mxsolver;

double totalTimeEncoding;
double totalTimeSolving;
double initial_time;
double begin_solving_time;
double begin_encoding_time;
double end_solving_time;
double end_encoding_time;

static void SIGINT_exit(int signum) {
  double control_time = cpuTime();
  if(begin_encoding_time > end_encoding_time){
    printf("te %.2lf\n",control_time-begin_encoding_time+totalTimeEncoding);
    printf("ts %lf\n",totalTimeSolving);
  } else {
    printf("te %.2lf\n",totalTimeEncoding);
    printf("ts %.2lf\n",totalTimeSolving+control_time-begin_solving_time);
  }
  printf("rt %lf\n",control_time-initial_time);
  mxsolver->printAnswer(_UNKNOWN_);
  exit(_SATISFIABLE_);
}

//=================================================================================================
#if !defined(_MSC_VER) && !defined(__MINGW32__)
void limitMemory(uint64_t max_mem_mb)
{
// FIXME: OpenBSD does not support RLIMIT_AS. Not sure how well RLIMIT_DATA works instead.
#if defined(__OpenBSD__)
#define RLIMIT_AS RLIMIT_DATA
#endif

    // Set limit on virtual memory:
    if (max_mem_mb != 0){
        rlim_t new_mem_lim = (rlim_t)max_mem_mb * 1024*1024;
        rlimit rl;
        getrlimit(RLIMIT_AS, &rl);
        if (rl.rlim_max == RLIM_INFINITY || new_mem_lim < rl.rlim_max){
            rl.rlim_cur = new_mem_lim;
            if (setrlimit(RLIMIT_AS, &rl) == -1)
                printf("c WARNING! Could not set resource limit: Virtual memory.\n");
        }
    }

#if defined(__OpenBSD__)
#undef RLIMIT_AS
#endif
}
#else
void limitMemory(uint64_t /*max_mem_mb*/)
{
    printf("c WARNING! Memory limit not supported on this architecture.\n");
}
#endif


#if !defined(_MSC_VER) && !defined(__MINGW32__)
void limitTime(uint32_t max_cpu_time)
{
    if (max_cpu_time != 0){
        rlimit rl;
        getrlimit(RLIMIT_CPU, &rl);
        if (rl.rlim_max == RLIM_INFINITY || (rlim_t)max_cpu_time < rl.rlim_max){
            rl.rlim_cur = max_cpu_time;
            if (setrlimit(RLIMIT_CPU, &rl) == -1)
                printf("c WARNING! Could not set resource limit: CPU-time.\n");
        }
    }
}
#else
void limitTime(uint32_t /*max_cpu_time*/)
{
    printf("c WARNING! CPU-time limit not supported on this architecture.\n");
}
#endif

pair<int,int> encodeAndSolve(MAPFProblem &p, MAPFEncoder &e, int bound, int encoding, int amoEncoding, string outputPrefix, bool soc_optimal_bound, int problemFormulation){
  double control_time;
  begin_encoding_time = cpuTime();
  MaxSATFormula *maxsat_formula = e.encodeForOpenWBO(p,bound,encoding,amoEncoding,problemFormulation);
  printf("c Encoded with bound:%20d\n", bound);
  printf("c Number of variables:%19d\n", maxsat_formula->nVars());
  printf("c Number of hard clauses:%16d\n", maxsat_formula->nHard());
  printf("c Number of soft clauses:%16d\n", maxsat_formula->nSoft());
  printf("c Base cost is:%26d\n", e.baseCost);
  end_encoding_time = cpuTime();
  totalTimeEncoding += end_encoding_time-begin_encoding_time;
  printf("c Elapsed time[s]:%23.2f\n", end_encoding_time - initial_time);
  mxsolver->setInitialTime(end_encoding_time);
  begin_solving_time = cpuTime();
  mxsolver->loadFormula(maxsat_formula);
  printf("c Formula loaded\n");
  if(p.solution.size()>0){
    vec<Lit> partialModel;
    e.getModelFromSolution(p.solution,partialModel);
    mxsolver->loadInitialPartialModel(partialModel);
  }
  int ret = (int)mxsolver->search();
  printf("c Search finished\n");
  if(ret==_UNSATISFIABLE_)
    printf("c Solved with result: UNSAT\n");
  else if(ret==_OPTIMUM_)
    printf("c Solved with result: OPTIMUM\n");
  else if(ret==_SATISFIABLE_)
    printf("c Solved with result: SAT\n");
  else
    printf("c Solved with result: %d\n",ret);      

  end_solving_time = cpuTime();
  totalTimeSolving += end_solving_time-begin_solving_time;

  pair<int,int> result = make_pair(ret,INT_MAX);
  if ( ret == _OPTIMUM_ ) {
    outputPrefix += soc_optimal_bound?string("_soc_optimal.sol"):string("_makespan_optimal.sol");
    int cost = e.decodeFromOpenWBO(p,bound,mxsolver,outputPrefix);
    result.second = cost;
    printf("s Solution with cost %d printed to file\n",cost);
    control_time = cpuTime();
    printf("c Elapsed time[s]:%23.2f\n", control_time - initial_time);
    //    printf("soc %d\n",cost);		
  }
  printf("c Freeing formula memory\n");
  //delete(maxsat_formula);
  return result;
}

void copyMakespanFileToSOCFile(string prefix){
  string command=string("cp ")+prefix+string("_makespan_optimal.sol ")+prefix+string("_soc_optimal.sol");
  system(command.c_str());
}

MaxSAT* createMaxSATSolver(int algorithm,int cardinality, int graph_type,int verbosity ){
  MaxSAT *S = NULL;
  switch(algorithm){
  case _ALGORITHM_PART_MSU3_:
    S = new PartMSU3(verbosity, _PART_BINARY_, graph_type,cardinality);
    break;
  case _ALGORITHM_LINEAR_SU_:
    S = new LinearSU(verbosity,true,cardinality, 1);
    break;
  case _ALGORITHM_MSU3_:
    S = new MSU3(verbosity);
    break;
  case _ALGORITHM_OLL_:
    S = new OLL(verbosity, cardinality);
    break;
  default:
    printf("ERROR: Algorithm not availble\n");
    return(0);
  }

 S->setPrint(true);
 return(S);
}


//=================================================================================================
// Main:
int main(int argc, char **argv) {
  printf("c\nc MAPTFtoMaxSAT:\t MAPF solver through MaxSAT, using Open-WBO solver)\n");
  printf("c Version:\t\t December 2020 -- Release: 1.0\n");
  printf("c Authors:\t\t Roberto Asin, Jorge Baier, Rodrigo Lopez, Sebastian Hagedorn\n");
  printf("c Contact:\t\t rasin@inf.udec.cl\n");
  printf("c Open-WBO Version:\t September 2018 -- Release: 2.1\n");
  printf("c Open-WBO Authors:\t Ruben Martins, Vasco Manquinho, Ines Lynce\n");
  printf("c Open-WBO Contributors: Miguel Neves, Saurabh Joshi, Norbert Manthey, Mikolas Janota\n");
  try {
    NSPACE::setUsageHelp("c USAGE: %s [options] <input-file> <MAPF-result-file>\n\n");

#if defined(__linux__)
    fpu_control_t oldcw, newcw;
    _FPU_GETCW(oldcw);
    newcw = (oldcw & ~_FPU_EXTENDED) | _FPU_DOUBLE;
    _FPU_SETCW(newcw);
    printf(
        "c\nc WARNING: for repeatability, setting FPU to use double precision\n");
#endif

    IntOption verbosity("Open-WBO", "verbosity","Verbosity level (0=minimal, 1=more).\n", 0, IntRange(0, 1));
    IntOption cpu_lim("Open-WBO", "cpu-lim","Limit on CPU time allowed in seconds.\n", 0, IntRange(0, INT32_MAX));
    IntOption mem_lim("Open-WBO", "mem-lim", "Limit on memory usage in megabytes.\n", 0, IntRange(0, INT32_MAX));
    IntOption algorithm1("Open-WBO", "algorithm1","Search algorithm (1=linear-su,2=msu3,3=part-msu3,4=oll).\n", 2, IntRange(1, 4));
    IntOption algorithm2("Open-WBO", "algorithm2","Search algorithm (0=wbo,1=linear-su,2=msu3,3=part-msu3,4=oll,5=best).\n", 2, IntRange(0, 5));
    IntOption cardinality("Encodings", "cardinality","Cardinality encoding (0=cardinality networks, 1=totalizer, 2=modulo totalizer).\n", 1, IntRange(0, 2));
    IntOption encoding("MAPF encoding", "encoding", "MAPF encoding (0=FULL,1=MINIMAL1,2=MINIMAL2).\n", 0, IntRange(0, 2));
    IntOption amoEncoding("At-most one encoding", "amoEncoding", "AMO encoding (0=pairwise,1=seqc_enc,2=sortn_enc,3=cardn_enc,4=bitwise_enc,5=ladd_enc,6=tot_enc,7=mtot_enc,8=kmtot).\n", 4, IntRange(0, 8));
    IntOption graph_type("PartMSU3", "graph-type","Graph type (0=vig, 1=cvig, 2=res) (only for unsat-based partition algorithms).",0, IntRange(0, 2));
    IntOption problemFormulation("Problem formulation", "problemFormulation", "Formulation of the problem (0=without follow conflict,1=with follow conflict,).\n", 0, IntRange(0, 1));
    parseOptions(argc, argv, true);
    printf("c\nc===============================SOLVING CONFIGURATION===============================\n");
    printf("c\tProblem formulation:");
    switch(problemFormulation){
    case 0: printf("\t\tWithout considering follow conflicts.\n");break;
    case 1: printf("\t\tConsidering follow conflicts\n");
    }

    printf("c\tMAPF encoding:");
    switch(encoding){
    case 0: printf("\t\t\tFull redundant encoding\n");break;
    case 1: printf("\t\t\tMinimal with step-by-step constraints\n");break;
    case 2: printf("\t\t\tMinimal imposing agents stay at exacly one spot at a time\n");
    }
    
    printf("c\tMaxSAT Algorithm Phase1:");
    switch(algorithm1){
    case 1: printf("\tLinear SU\n");break;
    case 2: printf("\tMSU3\n");break;
    case 3: printf("\tPART-MSU3 with graph_type");
      switch(graph_type){
      case 0: printf("vig\n");break;
      case 1: printf("cvig\n");break;
      case 2: printf("res\n");break;
      }
    case 4: printf("\tOLL\n");
    }

    printf("c\tMaxSAT Algorithm Phase2:");
    switch(algorithm2){
    case 1: printf("\tLinear SU\n");break;
    case 2: printf("\tMSU3\n");break;
    case 3: printf("\tPART-MSU3 with graph_type");
      switch(graph_type){
      case 0: printf("vig\n");break;
      case 1: printf("cvig\n");break;
      case 2: printf("res\n");break;
      }
    case 4: printf("\tOLL\n");
    }

    printf("c\tAt-most one encoding:");
    switch(encoding){
    case 0: printf("\t\tPairwise\n");break;
    case 1: printf("\t\tSequencial Counter\n");break;
    case 2: printf("\t\tSorting Networks\n");break;
    case 3: printf("\t\tCardinality Networks\n");break;
    case 4: printf("\t\tBitwise\n");break;
    case 5: printf("\t\tAdder Networks\n");break;      
    case 6: printf("\t\tTotalizer\n");break;
    case 7: printf("\t\tM-Totalizer\n");break;
    case 8: printf("\t\tKM-Totalizer\n");
    }
    printf("c===================================================================================\nc\n");
    // Try to set resource limits:
    if (cpu_lim != 0) limitTime(cpu_lim);
    if (mem_lim != 0) limitMemory(mem_lim);

    initial_time = cpuTime();
    MaxSAT *S = NULL;
    totalTimeEncoding = 0;
    totalTimeSolving = 0;

    signal(SIGXCPU, SIGINT_exit);
    signal(SIGTERM, SIGINT_exit);
    signal(SIGINT, SIGINT_exit);

    if (argc <= 2) {
      printf("c Error: no filename or no resultFile.\n");
    }

    MAPFProblem p(argv[1]);
    MAPFEncoder e;
    int bound = p.getInitialBound();

    while(1){
      mxsolver = createMaxSATSolver(algorithm1,cardinality,graph_type,verbosity);
      pair<int,int> ret = encodeAndSolve(p,e,bound,encoding,amoEncoding,argv[2],false,problemFormulation);
      delete(mxsolver);
      if ( ret.first == _OPTIMUM_ ) {
	int opt_bound = p.getOptimumMakespan(ret.second);
	if(opt_bound <= bound){
	  copyMakespanFileToSOCFile(argv[2]);
	}else{
	  mxsolver = createMaxSATSolver(algorithm2,cardinality,graph_type,verbosity);
	  ret = encodeAndSolve(p,e,opt_bound,encoding,amoEncoding,argv[2],true,problemFormulation);
	  delete(mxsolver);
	}
	break;
      }
      bound++;
    }
    printf("te %.2lf\n",totalTimeEncoding);
    printf("ts %.2lf\n",totalTimeSolving);
    printf("rt %.2lf\n",cpuTime()-initial_time);
  } catch (OutOfMemoryException &) {
    sleep(1);
    printf("c Error: Out of memory.\n");
    printf("s UNKNOWN\n");
    exit(_ERROR_);
  } catch(MaxSATException &e) {
    sleep(1);
    printf("c Error: MaxSAT Exception: %s\n", e.getMsg());
    printf("s UNKNOWN\n");
    exit(_ERROR_);
  }
  return 0;
}
