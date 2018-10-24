#include <iostream>
#include <iomanip>
#include <cstdint>

#include "color/color.hpp"

int main( int argc, char *argv[] )
 {
  // Instead of float you may put std::uint8_t,std::uint16_t, std::uint32_t, std::uint64_t, double, long double
  color::gray<float> c;

  // initialize c before get. Actually this is gray version of turquoise
  c = color::constant::turquoise_t{};

  // Here is how to get gray from gray.
  auto g = color::get::gray( c );

  // Now do whatever you wan to do
  std::cout << g << std::endl;

  return EXIT_SUCCESS;
 }
