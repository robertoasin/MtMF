#include <assert.h>
#include "../cardenc/clset.hh"
#include "../cardenc/card.hh"
#include "../MaxSAT.h"
#include "../MaxTypes.h"
#include "../MaxSATFormula.h"
#include <vector>
#include <map>
#include <string>
#include <cstdio>
#include <cstring>
#include <cstdlib>
#include <utility>
#include <iostream>
#include <queue>
#include <climits>
using namespace openwbo;
using namespace std;

#define CHECK_REACHABILITY 1
#define DIJKSTRA 1

#define STAY 0
#define UP 1
#define DOWN 2
#define LEFT 3
#define RIGHT 4
#define INFINITE INT_MAX

struct Assignment{
  int type; //0 for onVars and 1 for ShiftVars
  int xPos;
  int yPos;
  int agent;
  int t;
  int op;
  
  Assignment(int typeArg, int xArg, int yArg, int agArg, int tArg, int opArg):type(typeArg),xPos(xArg),yPos(yArg),agent(agArg),t(tArg),op(opArg){ }
 };


struct MAPFProblem{
  struct Agent{
    int startX;
    int startY;
    int goalX;
    int goalY;
  };
  
  struct Cell{
    bool obstacle;
    vector<int> distanceToGoals;
    vector<int> distanceToStarts;
    vector<int> compatibleOperations;
  };
    
  int problemId;
  int nAgents;
  int nObstacles;
  int nOperations;
  int gridX,gridY;
  Cell** grid;
  int initialBound;
  int bestPossibleCost;
  vector< Agent > agentsInfo;
  vector< Assignment > solution;

  int getStartX(int a)       { return agentsInfo[a].startX;  }
  int getStartY(int a)       { return agentsInfo[a].startY; }
  int getGoalX(int a)        { return agentsInfo[a].goalX;   }
  int getGoalY(int a)        { return agentsInfo[a].goalY;   }
  bool obstacle(int x, int y){ return grid[x][y].obstacle;  }
  bool checkReachability(int x, int y, int a, int t, int bound){
#ifdef CHECK_REACHABILITY
    return grid[x][y].distanceToStarts[a]<=t and grid[x][y].distanceToGoals[a]<=(bound-t);
#else
    return 1;
#endif
  }
  int getShortestPathLength(int a){
    return grid[agentsInfo[a].startX][agentsInfo[a].startY].distanceToGoals[a];
  }
  vector<int> getCompatibleOperations(int x, int y){
    return grid[x][y].compatibleOperations;
  }

  pair<int,int> getNextPosWithOp(int x,int y, int o){
    assert(!grid[x][y].obstacle);
    switch(o){
    case STAY:
      return make_pair(x,y);
    case UP:
      assert(x>0 and !grid[x-1][y].obstacle);
      return make_pair(x-1,y);
    case DOWN:
      assert(x<gridX-1 and !grid[x+1][y].obstacle);
      return make_pair(x+1,y);
    case LEFT:
      assert(y>0 and !grid[x][y-1].obstacle);
      return make_pair(x,y-1);
    case RIGHT:
      assert(y<gridY-1 and !grid[x][y+1].obstacle);
      return make_pair(x,y+1);
    }
  }
  
  int getOpositeOperations(int o){
    switch(o){
    case STAY:  return STAY;
    case RIGHT: return LEFT;
    case LEFT:  return RIGHT;
    case UP:    return DOWN;
    case DOWN:  return UP;
    }
  } 
  
  MAPFProblem(string inputFile){
    nOperations = 5;
    FILE* f = fopen(inputFile.c_str(),"rt");
    char buffer[10001];
    char* out;
    //read header
    out = fgets(buffer,10000,f);
    sscanf(buffer,"%d",&problemId);
    //read Grid
    out = fgets(buffer,10000,f);
    //    assert(strcmp(buffer,"Grid:\n")==0);
    //read Grid Size
    out = fgets(buffer,10000,f);
    sscanf(buffer,"%d,%d",&gridX,&gridY);
    //read Grid chars
    nObstacles = 0;
    char gr[gridX][gridY];
    for(int x=0;x<gridX;++x){
      out = fgets(buffer,10000,f);
      for(int y=0;y<gridY;++y){
	gr[x][y] = buffer[y];
	if(gr[x][y]!='.') nObstacles++;
      }
    }
    //read Agents:
    out = fgets(buffer,10000,f);
    //    assert(strcmp(buffer,"Agents:")==0);
    //read nAgents
    out = fgets(buffer,10000,f);
    sscanf(buffer,"%d",&nAgents);
    agentsInfo.resize(nAgents);
    
    //reserve and initialize grid
    grid = (Cell**)malloc(gridX*sizeof(Cell*));
    for(int x=0;x<gridX;++x){
      grid[x] = new Cell[gridY];
      for(int y=0;y<gridY;++y){
	grid[x][y].obstacle = (gr[x][y]!='.');
	grid[x][y].distanceToGoals.resize(nAgents);
	grid[x][y].distanceToStarts.resize(nAgents);
	for(int a=0;a<nAgents;++a){
	  grid[x][y].distanceToGoals[a]  = INFINITE;
	  grid[x][y].distanceToStarts[a] = INFINITE;
	}
      }
    }

    //read agents info
    for(int a=0;a<nAgents;++a){
      out = fgets(buffer,10000,f);
      int id,sX,sY,gX,gY;
      sscanf(buffer,"%d,%d,%d,%d,%d",&id,&sX,&sY,&gX,&gY);
      agentsInfo[id].startX = gX;
      agentsInfo[id].startY = gY;
      agentsInfo[id].goalX  = sX;
      agentsInfo[id].goalY  = sY;
    }
    fclose(f);
    //    printf("Finished reading\n");
    computeCompatibleOperations();
    computeShortestPaths();
    //    printf("MAPFProblem read\n");
  }

  int getInitialBound(){
    return initialBound;
  }

  int getOptimumMakespan(int makespanOptimalCost){
    return(initialBound+makespanOptimalCost-bestPossibleCost-1);
  }

private:
  void computeCompatibleOperations(){
    for(int x=0;x<gridX;++x){
      for(int y=0;y<gridY;++y){
	if(not grid[x][y].obstacle){
	  grid[x][y].compatibleOperations.push_back(STAY);
	  if( x>0 and !grid[x-1][y].obstacle ) grid[x][y].compatibleOperations.push_back(UP);
	  if( x<gridX-1 and !grid[x+1][y].obstacle ) grid[x][y].compatibleOperations.push_back(DOWN);
	  if( y>0 and !grid[x][y-1].obstacle ) grid[x][y].compatibleOperations.push_back(LEFT);
	  if( y<gridY-1 and !grid[x][y+1].obstacle ) grid[x][y].compatibleOperations.push_back(RIGHT);
	}
      }
    }
  }

  void computeShortestPathsEuclidean(){
    initialBound = 1;
    bestPossibleCost = 0;
    for(int x=0;x<gridX;++x){
      for(int y=0;y<gridY;++y){
	for(int a=0;a<nAgents;++a){
	  grid[x][y].distanceToStarts[a] = fabs(x-agentsInfo[a].startX)+fabs(y-agentsInfo[a].startY);
	  grid[x][y].distanceToGoals[a]  = fabs(x-agentsInfo[a].goalX)+fabs(y-agentsInfo[a].goalY);
	}
      }
    }
    for(int a=0;a<nAgents;++a){
      bestPossibleCost+= grid[agentsInfo[a].goalX][agentsInfo[a].goalY].distanceToStarts[a];
      if(grid[agentsInfo[a].goalX][agentsInfo[a].goalY].distanceToStarts[a] > initialBound){
	initialBound = grid[agentsInfo[a].goalX][agentsInfo[a].goalY].distanceToStarts[a];
      }
    }
  }

  void computeShortestPathsDijkstra(){

    for(int x=0;x<gridX;++x){
      for(int y=0;y<gridY;++y){
	for(int a=0;a<nAgents;++a){
	  grid[x][y].distanceToStarts[a]=INT_MAX;
	  grid[x][y].distanceToGoals[a]=INT_MAX;
	}
      }
    }
    
    priority_queue< pair<int, pair<int,int> > > q;
    initialBound = 1;
    bestPossibleCost = 0;
    for(int a=0;a<nAgents;++a){
      //for start
      pair<int, pair<int,int> > s = make_pair(0, make_pair(agentsInfo[a].startX,agentsInfo[a].startY));//is a maxheap, so we play with negatives
      q.push(s);
      grid[agentsInfo[a].startX][agentsInfo[a].startY].distanceToStarts[a] = 0;
      while(not q.empty()){
	pair<int, pair<int,int> > node = q.top();
	q.pop();
	vector<int> operations = getCompatibleOperations(node.second.first,node.second.second);
	for(int o:operations){
	  pair<int,int> child = getNextPosWithOp(node.second.first,node.second.second,o);
	  if(grid[child.first][child.second].distanceToStarts[a]==INT_MAX){
	    grid[child.first][child.second].distanceToStarts[a] = -(node.first-1);
	    q.push(make_pair(node.first-1,child));
	  }else{
	    assert(grid[child.first][child.second].distanceToStarts[a]<=-(node.first-1));
	  }
	}
      }
      //for goal
      pair<int, pair<int,int> > g = make_pair(0, make_pair(agentsInfo[a].goalX,agentsInfo[a].goalY));
      grid[agentsInfo[a].goalX][agentsInfo[a].goalY].distanceToGoals[a] = 0;
      q.push(g);
      while(not q.empty()){
	pair<int, pair<int,int> > node = q.top();
	q.pop();
	vector<int> operations = getCompatibleOperations(node.second.first,node.second.second);
	for(int o:operations){
	  pair<int,int> child = getNextPosWithOp(node.second.first,node.second.second,o);
	  if(grid[child.first][child.second].distanceToGoals[a]==INT_MAX){
	    grid[child.first][child.second].distanceToGoals[a] = -(node.first-1);
	    q.push(make_pair(node.first-1,child));
	  }else{
	    assert(grid[child.first][child.second].distanceToGoals[a]<=-(node.first-1));
	  }
	}	
      }
      bestPossibleCost+= grid[agentsInfo[a].goalX][agentsInfo[a].goalY].distanceToStarts[a];
      if(grid[agentsInfo[a].goalX][agentsInfo[a].goalY].distanceToStarts[a]>initialBound){
	initialBound = grid[agentsInfo[a].goalX][agentsInfo[a].goalY].distanceToStarts[a];
      }
    }
  }
  void computeShortestPaths(){
    if(DIJKSTRA) computeShortestPathsDijkstra();
    else computeShortestPathsEuclidean();
  }
};

struct MAPFEncoder{

  struct VarInfo{
    int type,x,y,a,t;
    VarInfo(int pty, int px,int py,int pa,int pt) : type(pty),x(px),y(py),a(pa),t(pt){}
    VarInfo(){
      type=-1;
      x=-1;
      y=-1;
      a=-1;
      t=-1;
    }
  };
  
  int**** onVars;
  int****  shiftVars;
  int**   finalStateVars;

  map<int,VarInfo> reverseDict;
  int numVars;
  int lastOnVar;
  int lastShiftVar;
  int lastFinalStateVar;
  int numSoftClauses;
  int baseCost;
  bool unsatDetected;
  ClauseSet clauses;
  MAPFProblem *prob;

  MAPFEncoder(){
    prob = NULL;
  }

  void clean(){
    //    printf("Cleaning memory\n");
    if (prob==NULL) return;
    if (onVars!=NULL){
      for(int x=0;x<prob->gridX;++x){
	for(int y=0;y<prob->gridY;++y){
	  for(int a=0;a<prob->nAgents;++a){
	    free(onVars[x][y][a]);
	  }
	  free(onVars[x][y]);
	}
	free(onVars[x]);
      }
      free(onVars);
      onVars = NULL;
    }
    if(shiftVars!=NULL){
      for(int x=0;x<prob->gridX;++x){
	for(int y=0;y<prob->gridY;++y){
	  for(int o=0;o<prob->nOperations;++o){
	    free(shiftVars[x][y][o]);
	  }
	  free(shiftVars[x][y]);
	}
	free(shiftVars[x]);
      }
      free(shiftVars);
      shiftVars=NULL;
    }
    if(finalStateVars!=NULL){
      for(int a=0;a<prob->nAgents;++a){
	free(finalStateVars[a]);
      }
      free(finalStateVars);
      finalStateVars=NULL;
    }
    printf("c Memory cleaned\n");
  }

  void reserveMemory(MAPFProblem &p, int bound){
    clean();
    prob = &p;
    //    printf("Reserving memory for map size (%d,%d), with %d agents and bound %d\n",p.gridX,p.gridY,p.nAgents,bound);
    onVars = (int****)malloc(p.gridX*sizeof(int***));
    for(int x=0;x<p.gridX;++x){
      onVars[x] = (int***)malloc(p.gridY*sizeof(int**));
      for(int y=0;y<p.gridY;++y){
	onVars[x][y]=(int**)malloc(p.nAgents*sizeof(int*));
	for(int a=0;a<p.nAgents;++a){
	  onVars[x][y][a]=(int*)malloc((bound+1)*sizeof(int));
	  for(int t=0;t<=bound;++t){
	    onVars[x][y][a][t] = 0;
	  }
	}
      }
    }

    shiftVars = (int****)malloc(p.gridX*sizeof(int***));
    for(int x=0;x<p.gridX;++x){
      shiftVars[x] = (int***)malloc(p.gridY*sizeof(int**));
      for(int y=0;y<p.gridY;++y){
	shiftVars[x][y]=(int**)malloc(p.nOperations*sizeof(int*));
	for(int o=0;o<p.nOperations;++o){
	  shiftVars[x][y][o]=(int*)malloc(bound*sizeof(int));
	  for(int t=0;t<bound;++t){
	    shiftVars[x][y][o][t] = 0;
	  }
	}
      }
    }

    finalStateVars = (int**)malloc(p.nAgents*sizeof(int*));
    for(int a=0;a<p.nAgents;++a){
      finalStateVars[a] = (int*)malloc((bound+1)*sizeof(int));
      for(int t=0;t<=bound;++t){
	finalStateVars[a][t] = 0;
      }
    }
  }
  
  void createVars(MAPFProblem &p, int bound){
    numVars = 0;
    lastOnVar = 0;
    lastShiftVar = 0;
    lastFinalStateVar = 0;
    reserveMemory(p,bound);
    //    printf("memory reserved\n");
    //Create on vars
    for(int x=0;x<p.gridX;++x){
      for(int y=0;y<p.gridY;++y){
	if(not p.obstacle(x,y)){
	  for(int a=0;a<p.nAgents;++a){
	    for(int t=0;t<=bound;++t){
	      if(p.checkReachability(x,y,a,t,bound)){
		numVars++;
		onVars[x][y][a][t] = numVars;
		reverseDict[numVars] = VarInfo(0,x,y,a,t);
		if(t==0 and (x!=p.agentsInfo[a].startX or y!=p.agentsInfo[a].startY)){
		  printf("THIS SHOULDN'T HAPPEN\n");
		  assert(false);
		}
	      }
	    }
	  }
	}
      }
    }
    lastOnVar = numVars;

    //Create shift vars
    for(int x=0;x<p.gridX;++x){
      for(int y=0;y<p.gridY;++y){
	if(not p.obstacle(x,y)){
	  vector<int> op = p.getCompatibleOperations(x,y);
	  for(int o:op){
	    for(int t=0;t<bound;++t){
	      numVars++;
	      shiftVars[x][y][o][t] = numVars;
	      reverseDict[numVars] = VarInfo(2,x,y,o,t);
	    }
	  }
	}
      }
    }
    lastShiftVar = numVars;

    //Create finalState vars
    baseCost = 0;
    for(int a=0;a<p.nAgents;++a){
      baseCost += p.getShortestPathLength(a);
      for(int t=p.getShortestPathLength(a);t<=bound;++t){
	numVars++;
	finalStateVars[a][t] = numVars;
	reverseDict[numVars] = VarInfo(1,0,0,a,t);
      }
    }
    lastFinalStateVar = numVars;
    numSoftClauses = numVars - lastShiftVar - p.nAgents;
  }


  void printWCNFFormula(MAPFProblem &p, int bound, string outputFileName){
    //    printf("Printing WCNF formula\n");
    FILE* f = fopen(outputFileName.c_str(),"wt");
    if(unsatDetected){
      //print formula with just empty clause
      fprintf(f,"p wcnf 0 1 2\n");
      fprintf(f,"2 0\n");
    }else{
      fprintf(f,"p wcnf %d %d %d\n",numVars,(int)clauses.clauses.size()+numSoftClauses,numSoftClauses);
      for(int a=0;a<p.nAgents;++a){
	for(int t=p.getShortestPathLength(a);t<bound;++t){
	  fprintf(f,"1 %d 0\n",finalStateVars[a][t]);
	}
      }
      for(int c=0;c<clauses.clauses.size();++c){
	fprintf(f,"%d ",numSoftClauses);
	for(int l:clauses.clauses[c]){
	  fprintf(f,"%d ",l);
	}
	fprintf(f,"0\n");
      }
    }
    fclose(f);
  }
  
  void encodeHard(MAPFProblem &p, int bound, int encoding, int amoEncoding, int problemFormulation){
    clauses.clear();
    //    printf("encoding\n");
    unsatDetected=false;
    createVars(p,bound);
    //    printf("vars created\n");
    //relate on and finalState vars
    int baseCost=0;
    for(int a=0;a<p.nAgents;++a){
      baseCost+=p.getShortestPathLength(a);
      for(int t=p.getShortestPathLength(a);t<bound;++t){
	int lit1 = finalStateVars[a][t];
	int lit2 = finalStateVars[a][t+1];
	int lit3 = onVars[p.getGoalX(a)][p.getGoalY(a)][a][t];
	assert(lit1!=0 and lit2!=0 and lit3!=0);
	vector<int> c1 {-lit3,-lit2, lit1};
	vector<int> c2 {lit2, -lit1};
	vector<int> c3 {lit3, -lit1};
	clauses.create_clause(c1);
	clauses.create_clause(c2);
	clauses.create_clause(c3);
      }
    }
    //printf("on and finalState vars relationship created\n");
    //relate on and shift vars
    //if on(a,x,y,t) and shift(x,y,o,t) => on(a,x1,y1,t+1)
    //if on(a,x,y,t) and on(a,x1,y1,t+1) => shift(x,y,o,t) 
    for(int x=0;x<p.gridX;++x){
      for(int y=0;y<p.gridY;++y){
	if(not p.obstacle(x,y)){
	  for(int a=0;a<p.nAgents;++a){
	    //printf("%d %d %d\n",x,y,a);
	    vector<int> operations = p.getCompatibleOperations(x,y);
	    for(int o:operations){
	      //printf("%d %d %d %d\n",x,y,a,o);
	      pair<int,int> pos = p.getNextPosWithOp(x,y,o);
	      int x1 = pos.first;
	      int y1 = pos.second;
	      for(int t=0;t<bound;++t){
		//printf("%d %d %d %d %d\n",x,y,a,o,t);
		if(p.checkReachability(x,y,a,t,bound)){
		  int lit1 = onVars[x][y][a][t];
		  int lit2 = shiftVars[x][y][o][t];
		  if(p.checkReachability(x1,y1,a,t+1,bound)){
		    int lit3 = onVars[x1][y1][a][t+1];
		    assert(lit1!=0 and lit2!=0 and lit3!=0);
		    vector<int> c1 {-lit1,-lit2,lit3};
		    vector<int> c2 {-lit1,lit2,-lit3};
		    clauses.create_clause(c1);
		    clauses.create_clause(c2);
		    //printf("clauses added\n");
		  }else{
		    vector<int> c1 {-lit1,-lit2};
		    clauses.create_clause(c1);
		  }
		}
		//printf("clauses maybe added\n");
	      }
	    }
	  }
	}
      }
    }
    //    printf("on and shift vars relationship created\n");
    
    //agents start at their initial positions at time 0
    //on(a,startX(a),startY(a),0)
    for(int a=0;a<p.nAgents;++a){
      int startX = p.getStartX(a);
      int startY = p.getStartY(a);
      if(onVars[startX][startY][a][0]==0){
	unsatDetected = true;
	return;
      }
      vector<int> c {onVars[startX][startY][a][0]};
      clauses.create_clause(c);
    }
    
    //agents should stay at their goal position at time bound
    //on(a,goalX(a),goalY(a),bound)
    for(int a=0;a<p.nAgents;++a){
      int goalX = p.getGoalX(a);
      int goalY = p.getGoalY(a);
      vector<int> c1 {onVars[goalX][goalY][a][bound]};
      vector<int> c2 {finalStateVars[a][bound]};
      clauses.create_clause(c1);
      clauses.create_clause(c2);
    }

    if(encoding!=2){
      //agents move or stay
      //on(a,x,y,t) -> on(a,x+1,y,t+1) v on(a,x-1,y,t+1) v on(a,x,y+1,t+1) v on(a,x,y-1,t+1) v on(a,x,y,t+1)
      //agents arrive to its place from somewhere possible
      //on(a,x,y,t) -> on(a,x+1,y,t-1) v on(a,x-1,y,t-1) v on(a,x,y+1,t-1) v on(a,x,y-1,t-1) v on(a,x,y,t-1)
      //this is redundant
      //works really badly if not on
      for(int x=0;x<p.gridX;++x){
	for(int y=0;y<p.gridY;++y){
	  if(not p.obstacle(x,y)){
	    for(int a=0;a<p.nAgents;++a){
	      for(int t=0;t<=bound;++t){
		if(p.checkReachability(x,y,a,t,bound)){
		  vector<int> operations = p.getCompatibleOperations(x,y);
		  vector<int> c1 {-onVars[x][y][a][t]};
		  vector<int> c2 {-onVars[x][y][a][t]};
		  
		  for(int o:operations){
		    pair<int,int> pos = p.getNextPosWithOp(x,y,o);
		    int x1 = pos.first;
		    int y1 = pos.second;
		    assert(not p.obstacle(x1,y1));
		    if(p.checkReachability(x1,y1,a,t+1,bound)){
		      c1.push_back(onVars[x1][y1][a][t+1]);
		    }
		    if(p.checkReachability(x1,y1,a,t-1,bound)){
		      c2.push_back(onVars[x1][y1][a][t-1]);
		    }
		  }
		  if(t<bound){
		    assert(c1.size()>0);
		    clauses.create_clause(c1);
		  }
		  if(t>0){
		    assert(c2.size()>0);
		    clauses.create_clause(c2);
		  }
		}
	      }
	    }
	  }
	}
      }
    }
    
    //no cross and not follow
    for(int x=0;x<p.gridX;++x){
      for(int y=0;y<p.gridY;++y){
	if(not p.obstacle(x,y)){
	  for(int t=0;t<bound;++t){
	    vector<int> clause;
	    vector<int> operations = p.getCompatibleOperations(x,y);
	    for(int o:operations){
	      clause.push_back(shiftVars[x][y][o][t]);
	      if(o!=STAY){		
		pair<int,int> pos = p.getNextPosWithOp(x,y,o);
		int x1 = pos.first;
		int y1 = pos.second;
		if(problemFormulation==1){
		  //if (x,y) shifts towards (x1,y1), (x1,y1) should stay to avoid crossing and/or following conflicts.
		  int lit1 = shiftVars[x][y][o][t];
		  int lit2 = shiftVars[x1][y1][STAY][t];
		  vector<int> c1 {-lit1, lit2};
		  clauses.create_clause(c1);
		}else{
		  assert(problemFormulation==0);
		  //if (x,y) shifts towards (x1,y1), (x1,y1) cannot shift towards (x,y).
		  int o2 = p.getOpositeOperations(o);
		  int lit1 = shiftVars[x][y][o][t];
		  int lit2 = shiftVars[x1][y1][o2][t];
		  vector<int> c1 {-lit1, lit2};
		  clauses.create_clause(c1);
		}
	      }
	    }
	    assert(clause.size()>0);
	    //cardinality_constraint here for exactly 1 shift var per time t
	    clauses.create_clause(clause);
	    if(clause.size()>1) _encode_atmost(clauses,clause,1,numVars,enc_exp);
	  }
	}
      }
    }

    int encType = amoEncoding;//enc_seqc;
    //int encType = enc_cardn;
    //int encType = enc_tot;
    if(encoding!=1){
      //agents must be at exactly one place at a time
      //is this redundant with h6 and h7?
      for(int a=0;a<p.nAgents;++a){
	//      printf("%d\n",a);
	for(int t=1;t<=bound;++t){//t0 agent is at start
	  vector<int> clause;
	  for(int x=0;x<p.gridX;++x){
	    for(int y=0;y<p.gridY;++y){
	      if( not p.obstacle(x,y) and
		  p.checkReachability(x,y,a,t,bound) ){
		clause.push_back(onVars[x][y][a][t]);
	      }
	    }
	  }
	  assert(clause.size()>0);
	  clauses.create_clause(clause);
	  _encode_atmost(clauses,clause,1,numVars,encType);
	}
      }
    }

    //each position hosts at most one agent at a time
    for(int x=0;x<p.gridX;++x){
      for(int y=0;y<p.gridY;++y){
	if(not p.obstacle(x,y)){
	  for(int t=0;t<=bound;++t){
	    vector<int> clause;
	    for(int a=0;a<p.nAgents;++a){
	      if(p.checkReachability(x,y,a,t,bound)){
		clause.push_back(onVars[x][y][a][t]);
	      }
	    }
	    if(clause.size()>1){
	      assert(clause.size()>1);
	      _encode_atmost(clauses,clause,1,numVars,encType);
	    }
	  }
	}
      }
    }
  }

  void encodeToFile(MAPFProblem &p, int bound, int encoding, string outputFileName, int amoEncoding, int problemFormulation){
    encodeHard(p,bound,encoding,amoEncoding,problemFormulation);
    printWCNFFormula(p,bound,outputFileName);
    //    printf("Base cost is %d\n",baseCost);
  }

  MaxSATFormula* encodeForOpenWBO(MAPFProblem &p, int bound, int encoding, int amoEncoding, int problemFormulation){
    encodeHard(p,bound,encoding,amoEncoding, problemFormulation);
    MaxSATFormula* f = new MaxSATFormula();
    f->setProblemType(_UNWEIGHTED_);
    f->setHardWeight(numSoftClauses);
    
    for(int a=0;a<p.nAgents;++a){
      for(int t=p.getShortestPathLength(a);t<bound;++t){
	vec<Lit> lits;
	int var = finalStateVars[a][t] - 1;
	while (var >= f->nVars()) f->newVar();
	lits.push(mkLit(var));
	f->addSoftClause(1,lits);
      }
    }

    for(int c=0;c<clauses.clauses.size();++c){
      vec<Lit> lits;
      for(int l:clauses.clauses[c]){
	int var = abs(l) - 1;
	while (var >= f->nVars()) f->newVar();
	lits.push((l > 0) ? mkLit(var) : ~mkLit(var));
      }
      clauses.clauses[c].clear();
      f->addHardClause(lits);
    }
    clauses.clear();
    
    return(f);
  }

  void getModelFromSolution(vector< Assignment > &sol,vec<Lit> &partialModel){
    for(int i=0;i<sol.size();++i){
      if(sol[i].type==0){
	int var = onVars[sol[i].xPos][sol[i].yPos][sol[i].agent][sol[i].t]-1;
	partialModel.push(mkLit(var));
      }else if(sol[i].type==1){
	int var = shiftVars[sol[i].xPos][sol[i].yPos][sol[i].op][sol[i].t]-1;
	partialModel.push(mkLit(var));
      }
    }
  }

  
  int decodeToFile(MAPFProblem &p,int bound,string inputModel,string outputName){
    //    printf("decoding\n");
    createVars(p,bound);
    //    printf("vars created\n");
    FILE *f = fopen(inputModel.c_str(),"rt");
    FILE *o = fopen(outputName.c_str(),"wt");
    int lit;
    int cost = 0;
    vector<int> agentsCost(p.nAgents,bound);
    while(fscanf(f,"%d",&lit)!=EOF){
      if(lit>0 and lit<=lastOnVar){
	VarInfo v = reverseDict[lit];
	fprintf(o,"on(%d,%d,%d,%d)\n",v.a,v.x,v.y,v.t);
      }else if(lit>0 and lit>lastShiftVar and lit<=lastFinalStateVar){
	VarInfo v = reverseDict[lit];
	if(agentsCost[v.a]>v.t){
	  agentsCost[v.a]=v.t;
	}
      }     
    }
    for(int a=0;a<p.nAgents;++a){
      cost+=agentsCost[a];
    }
    fclose(f);
    fclose(o);
    return(cost);
  }

  int decodeFromOpenWBO(MAPFProblem &p,int bound,MaxSAT* solver,string outputName){
    //    printf("decoding\n");
    createVars(p,bound);
    FILE *o = fopen(outputName.c_str(),"wt");
    vector<int> agentsCost(p.nAgents,bound);
    p.solution.clear();
    for(int i=0;i<numVars;++i){
      int lit = solver->getValue(i);
      if( lit > 0 and lit<=lastOnVar){
	VarInfo v = reverseDict[lit];
	p.solution.push_back(Assignment(0,v.x,v.y,v.a,v.t,-1));
	fprintf(o,"on(%d,%d,%d,%d)\n",v.a,v.x,v.y,v.t);
      }else if(lit>0 and lit>lastShiftVar and lit<=lastFinalStateVar){
	VarInfo v = reverseDict[lit];
	if(agentsCost[v.a]>v.t){
	  agentsCost[v.a]=v.t;
	}
      }else if(lit>0 and lit>lastOnVar and lit<=lastShiftVar){
	VarInfo v = reverseDict[lit];
	fprintf(o,"shift(%d,%d,%d,%d)\n",v.x,v.y,v.t,v.a);
      }
    }
    int cost = 0;
    for(int a=0;a<p.nAgents;++a){
      cost+=agentsCost[a];
    }
    //    printf("Total cost is %d\n",cost);
    fclose(o);
    return(cost);
  }
};




