#include "DynamicQuantityTests.h"
#include "Individual.h"
#include "SolutionValidator.h"
#include "InstanceCVRPLIB.h"
#include <cmath>
#include <stdexcept>

namespace {
int findDemand(const Params& p, double atLeast, int except = -1) {
 for (int i=1;i<(int)p.originalDemand.size();++i) if(i!=except && p.originalDemand[i]>=atLeast) return i;
 return -1;
}
Individual baseDynamic(const Params& p, int target) {
 Individual x(const_cast<Params&>(p)); x.quantityRepresentation=QuantityRepresentation::DYNAMIC_QUANTITY;
 x.routesDynamic.assign(p.originalDemand.size()+2, {});
 int route=0; for(int i=1;i<(int)p.originalDemand.size();++i) if(i!=target) x.routesDynamic[route++].push_back({i,p.originalDemand[i]});
 return x;
}
void require(bool value, const char* name) { if(!value) throw std::logic_error(name); }
}

bool runDynamicQuantityTests(Params& params, std::ostream& out)
{
 const int target=findDemand(params,40.0); require(target>0,"no demand-40 customer for dynamic tests");
 auto pass=[&](const char* n){out<<"[DYNAMIC-TEST] "<<n<<" PASS\n";};
 // Test 1 and 8: 30+10 delivery, evaluator/validator distance agreement.
 Individual feasible=baseDynamic(params,target); feasible.routesDynamic[100]={{target,30.0}}; feasible.routesDynamic[101]={{target,10.0}};
 feasible.evaluateDynamicQuantity(params); ValidationResult v=validateSolutionFromScratch(feasible,params);
 require(v.feasible(),"test1 split should be feasible"); require(std::fabs(v.distance-feasible.eval.distance)<=MY_EPSILON,"test8 distance mismatch"); pass("split_30_plus_10_and_distance");
 // Test 2 overdelivery.
 Individual over=baseDynamic(params,target); over.routesDynamic[100]={{target,30.0}}; over.routesDynamic[101]={{target,15.0}};
 require(!validateSolutionFromScratch(over,params).originalDemandValid,"test2 expected overdelivery"); pass("overdelivery");
 // Test 3 underdelivery.
 Individual under=baseDynamic(params,target); under.routesDynamic[100]={{target,30.0}};
 require(!validateSolutionFromScratch(under,params).originalDemandValid,"test3 expected underdelivery"); pass("underdelivery");
 // Test 4 duplicate visit on one route must fail even though total demand is exact.
 Individual duplicate=baseDynamic(params,target); duplicate.routesDynamic[100]={{target,20.0},{target,20.0}};
 require(!validateSolutionFromScratch(duplicate,params).visitBlocksValid,"test4 expected duplicate route visit"); pass("duplicate_original_in_route");
 // Test 5 capacity excess: choose two other customers so their combined load with target exceeds Q.
 int a=findDemand(params,50.0,target), b=findDemand(params,20.0,target); require(a>0&&b>0,"insufficient demands for capacity test");
 Individual cap=baseDynamic(params,target); cap.routesDynamic[100]={{target,40.0},{a,params.originalDemand[a]},{b,params.originalDemand[b]}};
 ValidationResult capV=validateSolutionFromScratch(cap,params); require(!capV.capacityFeasible,"test5 expected capacity failure"); pass("capacity");
 // Test 6 uses a private Params copy and restores the global conflict matrix afterwards.
 Params conflictParams=params; const auto savedConflict=conflict_matrix; conflictParams.originalCommodity[target]=1; conflictParams.originalCommodity[a]=2;
 conflict_matrix.assign(3,std::vector<bool>(3,false)); conflict_matrix[1][2]=conflict_matrix[2][1]=true;
 Individual conflict=baseDynamic(conflictParams,target); conflict.routesDynamic[100]={{target,40.0},{a,conflictParams.originalDemand[a]}};
 require(!validateSolutionFromScratch(conflict,conflictParams).commodityFeasible,"test6 expected commodity conflict"); conflict_matrix=savedConflict; pass("commodity_conflict");
 // Test 7: make target's latest time impossible in a private copy.
 Params twParams=params; twParams.originalLatest[target]=-1.0;
 Individual tw=baseDynamic(twParams,target); tw.routesDynamic[100]={{target,40.0}};
 require(!validateSolutionFromScratch(tw,twParams).timeWindowFeasible,"test7 expected time-window failure"); pass("time_window");
 // Deep value semantics: vector-owned allocations must not alias through copy/move.
 Individual copy=feasible; copy.routesDynamic[100][0].quantity=25.0; copy.routesDynamic[101][0].quantity=15.0;
 require(std::fabs(feasible.routesDynamic[100][0].quantity-30.0)<=MY_EPSILON,"copy alias");
 Individual assigned(params); assigned=feasible; assigned.routesDynamic[100][0].quantity=20.0;
 require(std::fabs(feasible.routesDynamic[100][0].quantity-30.0)<=MY_EPSILON,"assignment alias");
 Individual moved(std::move(copy)); require(std::fabs(moved.routesDynamic[100][0].quantity-25.0)<=MY_EPSILON,"move construction");
 Individual moveAssigned(params); moveAssigned=std::move(assigned); require(std::fabs(moveAssigned.routesDynamic[100][0].quantity-20.0)<=MY_EPSILON,"move assignment"); pass("copy_move_isolation");
	// Fixed -> dynamic conversion must preserve the fixed route semantics exactly.
	Individual fixed(params); for (auto& route : fixed.chromR) route.clear();
	for (int client=1; client<=params.nbClients; ++client) fixed.chromR[client-1].push_back(client);
	fixed.evaluateCompleteCost(params); const ValidationResult fixedCheck=validateSolutionFromScratch(fixed,params);
	require(fixedCheck.feasible(),"fixed conversion source invalid");
	fixed.convertFixedToDynamic(params); fixed.evaluateDynamicQuantity(params); const ValidationResult dynamicCheck=validateSolutionFromScratch(fixed,params);
	require(dynamicCheck.feasible(),"converted dynamic invalid");
	require(std::fabs(fixedCheck.distance-dynamicCheck.distance)<=MY_EPSILON,"fixed dynamic distance mismatch");
	require(std::fabs(fixedCheck.capacityExcess-dynamicCheck.capacityExcess)<=MY_EPSILON,"fixed dynamic capacity mismatch");
	require(std::fabs(fixedCheck.timeWindowExcess-dynamicCheck.timeWindowExcess)<=MY_EPSILON,"fixed dynamic TW mismatch");
	out << "[DYNAMIC-TEST] fixed_to_dynamic FixedDistance=" << fixedCheck.distance << " DynamicDistance=" << dynamicCheck.distance << " Difference=" << (dynamicCheck.distance-fixedCheck.distance) << " PASS\n";
	 out<<"[DYNAMIC-TEST] ALL PASS\n"; return true;
}
