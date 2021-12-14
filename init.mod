# Developed by Xingpeng.Li
# initial candidate line selection for
# Transmission Expansion Planning w. & w.o. Seasonal Network Optimization
# model init.mod;

reset;


###########################
#### set
set BUS;    # set of buses
set BRANCH; # set of branches
set GEN;   # Gen Data
set HOUR = {1..24};   # hour Data 1..24
set NEWLINE; # New candidate line data


#### PARAMETERS:
param nEpoch = 1;    # number of year-epoch in the planning horizon
param nYEpoch = 1;    # number of years in each epoch in the planning horizon
param nTday = 4;   # number of typical days in a year
set EPOCH = {1..nEpoch};  # epoch index in the planning horizon
set TDAY = {1..nTday};  # typical day in a year

param anlLdInc = 0.02;  # annual load increase
#param anlMaintainCostPct = 0.03;   # annual maintainance cost percentage against capital cost

# Bus Data
param bus_num        {BUS}; # Bus Number

# Load Data
param Pload {BUS, TDAY};
param LoadPct {HOUR, TDAY};

# GEN Data
param gen_bus	     {GEN}; # GEN location
param gen_Pmax      {GEN}; # Max gen production
param gen_Pmin      {GEN}; # Min gen production: all set to ZERO to avoid UC problem
param gen_Cost	     {GEN}; # Linear Cost Term

# Branch Data
param branch_fbus    {BRANCH}; # from bus for line
param branch_tbus    {BRANCH}; # to bus for line
param branch_x       {BRANCH}; # line reactance
param branch_rateA   {BRANCH}; # long term thermal rating

# NewLine DATA
param NL_fbus    {NEWLINE}; # from bus for line
param NL_tbus    {NEWLINE}; # to bus for line
param NL_x       {NEWLINE}; # line reactance
param NL_rateA   {NEWLINE}; # long term thermal rating
param NL_cost    {NEWLINE}; # One-time capital cost

# A large Number
param BigNum = 10^5;
param BaseMVA = 100;  # base MVA

#### VARIABLES:
var bus_angle {BUS, HOUR, TDAY, EPOCH};        # Variable for Bus Angles
var gen_supply {GEN, HOUR, TDAY, EPOCH};      # Variable for GEN Supply
var line_flow {BRANCH, HOUR, TDAY, EPOCH};     # Variable for all line flows
/*
var uNL {NEWLINE, EPOCH} binary;    # availability of new line
var vNL {NEWLINE, EPOCH} binary;    # status of new line construction season
var NL_flow {NEWLINE, HOUR, TDAY, EPOCH};  
*/
var GenCost;
#var InvestmentCost;

###########################
#### OBJECTIVE:
minimize TotalCost: GenCost; # + InvestmentCost;

#### CONSTRAINTS:
subject to GenCostCnstr: GenCost = nYEpoch*BaseMVA*365/nTday*sum{g in GEN, t in HOUR, d in TDAY, y in EPOCH} gen_supply[g,t,d,y]*gen_Cost[g];
#subject to CapitalCostCnstr: InvestmentCost = sum{k in NEWLINE, y in EPOCH}vNL[k,y]*NL_cost[k]*( 1+(nYEpoch-y+1)*anlMaintainCostPct*nYEpoch );

##--- POWER BALANCE EQUATION.
subject to PowerBal{n in BUS, t in HOUR, d in TDAY, y in EPOCH}: 		# Power Balance Constraint
       sum{j in BRANCH: branch_tbus[j] == n}line_flow[j,t,d,y]
	 - sum{j in BRANCH: branch_fbus[j] == n}line_flow[j,t,d,y]
#	 + sum{j in NEWLINE: branch_tbus[j] == n}NL_flow[j,t,d,y]
#	 - sum{j in NEWLINE: branch_fbus[j] == n}NL_flow[j,t,d,y]
	 + sum{g in GEN: gen_bus[g] == n}gen_supply[g,t,d,y] 
	 = Pload[n,d]*LoadPct[t,2]*((1+anlLdInc)^( (y-1)*nYEpoch) );

##--- POWER OUTPUT LIMITATION FOR GENERATORS.
subject to PGenMaxMin {g in GEN, t in HOUR, d in TDAY, y in EPOCH}: 		# Gen min & max constraint.
	            gen_Pmin[g] <= gen_supply[g,t,d,y] <= gen_Pmax[g];

##--- LINE FLOW EQUATION, AND THERMAL CONSTRAINTS FOR THE EXISTING LINES.
subject to Line_Flow_value{j in BRANCH, t in HOUR, d in TDAY, y in EPOCH}: 
                line_flow[j,t,d,y] = (bus_angle[branch_fbus[j],t,d,y] - bus_angle[branch_tbus[j],t,d,y])/branch_x[j];
				
subject to Thermal1{j in BRANCH, t in HOUR, d in TDAY, y in EPOCH}: 	# Thermal Constraint
	            -branch_rateA[j] <= line_flow[j,t,d,y];
				
subject to Thermal2{j in BRANCH, t in HOUR, d in TDAY, y in EPOCH}:	# Thermal Constraint 2  
	            branch_rateA[j] >= line_flow[j,t,d,y];
/*	
##--- LINE FLOW EQUATION, AND THERMAL CONSTRAINTS FOR THE NEW LINES.			
subject to NLine_Flow_value{i in NEWLINE, t in HOUR, d in TDAY, y in EPOCH}:
                NL_flow[i,t,d,y] - (bus_angle[NL_fbus[i],t,d,y] - bus_angle[NL_tbus[i],t,d,y])/NL_x[i] <= BigNum*(1-uNL[i,y]) ;

subject to NLine_invFlow_value{i in NEWLINE, t in HOUR, d in TDAY, y in EPOCH}:
                -(NL_flow[i,t,d,y] - (bus_angle[NL_fbus[i],t,d,y] - bus_angle[NL_tbus[i],t,d,y])/NL_x[i]) <= BigNum*(1-uNL[i,y]) ;
								 								 
subject to Thermal_NLine{j in NEWLINE, t in HOUR, d in TDAY, y in EPOCH}:
				-NL_flow[j,t,d,y] <= NL_rateA[j]*uNL[j,y];
				
subject to Thermal_NLine2{j in NEWLINE, t in HOUR, d in TDAY, y in EPOCH}:
				NL_flow[j,t,d,y] <= NL_rateA[j]*uNL[j,y];

subject to NL_Status_constraint{j in NEWLINE, y in EPOCH}:
                sum{yy in EPOCH: yy <= y}vNL[j,yy] <= uNL[j,y];

subject to NL_Status_constraint2{j in NEWLINE, y in EPOCH: y>1}:
                vNL[j,y] >= uNL[j,y] - uNL[j,y-1];

subject to NL_Status_constraint4{j in NEWLINE, y in EPOCH: y == 1}:
                vNL[j,y] = uNL[j,y];
*/			
###########################
#### Load data:
data data24.dat;

###########################
#----SCALE AND INITIALIZE THE DATA.
for{i in BUS, d in TDAY}
{
    let Pload[i,d] := Pload[i,d]/BaseMVA;
};

for{i in GEN}
{
    let gen_Pmax[i] := gen_Pmax[i]/BaseMVA;
    let gen_Pmin[i] := gen_Pmin[i]/BaseMVA;
};

for{i in BRANCH}
{
    let branch_rateA[i] := branch_rateA[i]/BaseMVA;
};

for{i in NEWLINE}
{
    let NL_rateA[i] := NL_rateA[i]/BaseMVA;
};

###########################
option solver gurobi;
option gurobi_options('mipgap=0.00 timelim=600 logfile=TEP_solverlog.txt');

#option solver cplexamp;
#option cplex_options('mipgap=0.01 integrality = 0.0 timelimit=90');

#let {j in NEWLINE, y in EPOCH} vNL[j,y] := 0;
#let {j in NEWLINE, y in EPOCH} uNL[j,y] := 0;

solve;


####### Show me the results.  Happy End!
display _total_solve_time;
display _total_solve_elapsed_time;
option show_stats 1;

#display {j in NEWLINE, y in EPOCH: vNL[j,y] == 1} vNL[j,y];
#display {j in NEWLINE, y in EPOCH: uNL[j,y] == 1} uNL[j,y];
#display sum{j in NEWLINE, y in EPOCH: vNL[j,y] == 1}vNL[j,y];
#display TotalCost, GenCost, InvestmentCost; 
#display bus_angle;
#display gen_supply;
#display line_flow;


# display {n in BUS, t in HOUR, d in TDAY, y in EPOCH: n==1 && y==1} Pload[n,d]*LoadPct[t]*((1+anlLdInc)^(y-1));
# display {g in GEN, t in HOUR, d in TDAY, y in EPOCH: g==1 && y==1} gen_supply[g,t,d,y];
# display {j in BRANCH, t in HOUR, d in TDAY, y in EPOCH: j==1 && y==1} line_flow[j,t,d,y];
# display {j in NEWLINE, t in HOUR, d in TDAY, y in EPOCH: j==1 && y==1} NL_flow[j,t,d,y];

param count; let count := 0;
printf "idx,branchIdx,RateA,Hr1,Hr2,Hr3,Hr4,Hr5,Hr6,Hr7,Hr8,Hr9,Hr10,Hr11,Hr12,Hr13,Hr14,Hr15,Hr16,Hr17,Hr18,Hr19,Hr20,Hr21,Hr22,Hr23,Hr24" > PFlowResults.csv;
printf ",Hr1,Hr2,Hr3,Hr4,Hr5,Hr6,Hr7,Hr8,Hr9,Hr10,Hr11,Hr12,Hr13,Hr14,Hr15,Hr16,Hr17,Hr18,Hr19,Hr20,Hr21,Hr22,Hr23,Hr24" > PFlowResults.csv;
printf ",Hr1,Hr2,Hr3,Hr4,Hr5,Hr6,Hr7,Hr8,Hr9,Hr10,Hr11,Hr12,Hr13,Hr14,Hr15,Hr16,Hr17,Hr18,Hr19,Hr20,Hr21,Hr22,Hr23,Hr24" > PFlowResults.csv;
printf ",Hr1,Hr2,Hr3,Hr4,Hr5,Hr6,Hr7,Hr8,Hr9,Hr10,Hr11,Hr12,Hr13,Hr14,Hr15,Hr16,Hr17,Hr18,Hr19,Hr20,Hr21,Hr22,Hr23,Hr24\n" > PFlowResults.csv;
for {k in BRANCH}
{
	let count := count + 1;
#	printf "%d,%d,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f\n", count,k,line_flow[k,t,1,1]
	printf "%d,%d,%f", count,k,branch_rateA[k]  > PFlowResults.csv;
	for {t in HOUR}
	{
		printf ",%f", abs(line_flow[k,t,1,1])  > PFlowResults.csv;
	}	
	for {t in HOUR}
	{
		printf ",%f", abs(line_flow[k,t,2,1])  > PFlowResults.csv;
	}
	for {t in HOUR}
	{
		printf ",%f", abs(line_flow[k,t,3,1])  > PFlowResults.csv;
	}
	for {t in HOUR}
	{
		printf ",%f", abs(line_flow[k,t,4,1])  > PFlowResults.csv;
	}
	printf "\n"  > PFlowResults.csv;
}






