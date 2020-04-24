#define BOOST_TEST_MODULE PIDTestModule

#include <iostream>
#include <fstream>
#include <boost/test/test_tools.hpp>
#include <boost/test/unit_test.hpp>

#include <boost/log/trivial.hpp>
#include <boost/log/common.hpp>
#include <boost/log/attributes.hpp>
#include <boost/log/utility/setup/from_stream.hpp>

#include "../src/PID.h"

namespace logging = boost::log;
namespace attrs = boost::log::attributes;

using namespace std;
using namespace boost;
using namespace boost::unit_test;

BOOST_AUTO_TEST_SUITE(PIDTestSuite)

    struct TestFixture
    {
        TestFixture() : i( 0 ) {

          // INITIALIZE LOGGER

          // Open the log settings file
          std::string logFileSettingsName = "../../logs/settings_for_test.txt";
          std::ifstream settings(logFileSettingsName);
          if (!settings.is_open())
          {
            std::cerr << "Could not open log settings file: " << logFileSettingsName << std::endl;
          }

          // Read the settings and initialize logging library
          logging::init_from_stream(settings);

          // Add some attributes
          logging::core::get()->add_global_attribute("TimeStamp", attrs::local_clock()); // each log line gets a timestamp
          logging::core::get()->add_global_attribute("LineID", attrs::counter<unsigned int>(1)); // lines are sequentially numbered

          // LOGGER INITIALIZED

          BOOST_LOG_TRIVIAL(info) << "Initialize Test";

        }

        ~TestFixture() {
          BOOST_LOG_TRIVIAL(info) << "Cleanup Test";
        }
        int i ;
    };


    BOOST_FIXTURE_TEST_CASE(BoostCheckTest, TestFixture)
    {
        // Provide a test predicate (i.e. a conditional statement) that evaluates
        // to true to allow the test to pass and will not indicate a failed test.

        PID pid;

        // Define PID gains
        double Kp = 0.1;      // Initial value for Kp
        double Ki = 0.001;    // Initial value for Ki
        double Kd = 2.8;      // Initial value for Kd

        // Define tuning variable
        bool do_tune = false;

        // Initialize PID
        pid.Init(Kp, Ki, Kd, do_tune);

        // Test Kp initialization
        BOOST_CHECK(pid.GetKp() == 2*Kp);

        // Test Ki initialization
        BOOST_CHECK(pid.GetKi() == Ki);

        // Test Kd initialization
        BOOST_CHECK(pid.GetKd() == Kd);

        // Test Kd initialization
        BOOST_CHECK(pid.GetTuneFlag() == do_tune);
    }

BOOST_AUTO_TEST_SUITE_END()

// More test suites

