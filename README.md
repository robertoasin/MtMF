# MAPFSAT: MaxSAT-based MAPF Solver
## Version 1.0 - December 2020

MAPFSAT is a MaxSAT-based MAPF Solver, using Open-WBO MaxSAT Solver.
Open-WBO was one of the best solvers in the partial MaxSAT categories at 
MaxSAT Evaluations 2014, 2015, 2016 and 2017 and in the decision and 
optimization for SMALLINT categories at PB Evaluation 2016.

Usage of the solver:
./mapfsat [options] <input-file> <output-file>

The following options are available in MAPFSAT:

## Global Options
### Problem Formulation (0=Without-follow, 1=With-follow)
```-problemFormulation      = <int32>  [   0 ..    1] (default: 0)```

### Verbosity level (0=minimal, 1=more)
```-verbosity    = <int32>  [   0 ..    1] (default: 1)```

### Search algorithm phase 1 (1=linear-su,2=msu3,3=part-msu3,4=oll)
```-algorithm1    = <int32>  [   1 ..    4] (default: 2)```

### Search algorithm phase 2 (1=linear-su,2=msu3,3=part-msu3,4=oll)
```-algorithm2    = <int32>  [   1 ..    4] (default: 1)```

### MAPF Encoding (0=FULL,1=MINIMAL1,2=MINIMAL2)
```-encoding      = <int32>  [   1 ..    3] (default: 1)```

### At-most-one encodings (0=pairwise, 1=seqc_enc, 2=sortn_enc, 3=cardn_enc, 4=bitwise_enc, 5=ladd_enc, 6=tot_enc, 7=mtot_enc, 8=kmtot)
```-amoEncoding          = <int32>  [   0 ..    8] (default: 4)```

### Cardinality encodings (0=cardinality networks, 1=totalizer, 2=modulo totalizer)
```-cardinality  = <int32>  [   0 ..    2] (default: 1)```
       
## PartMSU3 OPTIONS (algorithm=3, partition-based algorithm)
### Graph type (0=vig, 1=cvig, 2=res)
```-graph-type   = <int32>  [   0 ..    2] (default: 2)```

## Output of solver
Open-WBO follows the standard output of MaxSAT solvers:
* Comments ("c " lines) 
* Solution Status ("s " line):
  * s OPTIMUM FOUND : an optimum solution was found
  * s UNSATISFIABLE : the hard clauses are unsatisfiable
  * s SATISFIABLE   : a solution was found but optimality was not proven
* Solution Cost Line ("o " lines):
  * This represents the cost of the best solution found by the solver. The cost 
  of a solution is given by the sum of the weights of the unsatisfied soft clause.
* Solution Values (Truth Assignment) ("v " lines): 
  * This represents the truth assignment (true/false) assigned to each variable. 
  A literal is denoted by an integer that identifies the variable and the negation 
  of a literal is denoted by a minus sign immediately followed by the integer of 
  the variable.
  
> Authors: Roberto Asin, Jorge Baier, Rodrigo Lopez, Sebastian Hagedorn

> To contact the authors please send an email to:  rasin@inf.udec.cl

> Open-WBO Authors: Ruben Martins, Vasco Manquinho, Ines Lynce

> Open-WBO Contributors: Miguel Neves, Norbert Manthey, Saurabh Joshi, Mikolas Janota

> To contact Open-WBO the authors please send an email to:  open-wbo@sat.inesc-id.pt
