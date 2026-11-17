
#include "uw_robot_control/allocation/allocator.hpp"
#include <iomanip>
#include <iostream>
int main() {
  using namespace uw;
  AllocationGeometry g;g.rotating_sites=true;
  for(int j=0;j<3;++j)std::cin>>g.com[j];
  for(auto a:{&g.sites,&g.hinges,&g.axes,&g.child_centers})
    for(auto& v:*a)for(int j=0;j<3;++j)std::cin>>v[j];
  for(int i=0;i<4;++i)std::cin>>g.mass_fractions[i];
  double a,b;std::cout<<std::setprecision(17);
  while(std::cin>>a>>b) {
    auto matrix=wrench_matrix(V2(a,b),g);
    for(int i=0;i<6;++i)for(int j=0;j<4;++j)std::cout<<matrix(i,j)<<' ';
    std::cout<<'\n';
  }
}
