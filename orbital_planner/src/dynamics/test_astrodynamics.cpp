#include "catch.hpp"

#include <stdexcept>
#include "astrodynamics.hpp"


using namespace Astrodynamics;



TEST_CASE( "Test: helper functions around orbital elements", "[oe]" ) {

    StateOE valid_oe = {LEO_MAX, 0.2, 0.16, 0.28, 0.9, 0.8};
    StateOE invalid_oe = {-LEO_MAX, 0.2, 6.16, 0.28, 0.9, 8.8};
    State valid_eci = {-900979.5469268193, 958561.9738142684, 188849.79286785767, -15277.98248020127, -9996.980506656326, -869.1083959796933};


    CHECK( CheckOrbitalElements(valid_oe) == true );

    CHECK_THROWS_AS( CheckOrbitalElements(invalid_oe), std::runtime_error );


    auto res_eci = OE2ECI(valid_oe);
    for (int i = 0; i < res_eci.size(); ++i) {
        CHECK(res_eci[i] == Approx(valid_eci[i]).margin(0.0001));
    }

    auto res_oe = ECI2OE(valid_eci);
    for (int i = 0; i < res_oe.size(); ++i) {
        CHECK(res_oe[i] == Approx(valid_oe[i]).margin(0.0001));
    }




}



TEST_CASE( "Test: dynamics and jacobian functions", "[dynamics]" ) 
{
    // Test CW, central body, ..
    // Also test jacobian function values 



}




TEST_CASE( "Test: orbital sampling functions", "[sampling]" ) 
{




}
