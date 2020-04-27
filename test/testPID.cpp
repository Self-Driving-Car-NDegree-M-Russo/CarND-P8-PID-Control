#define BOOST_TEST_MODULE PIDTestModule

#include <iostream>
#include <fstream>
#include <boost/test/test_tools.hpp>
#include <boost/test/unit_test.hpp>

#include <boost/log/trivial.hpp>
#include <boost/log/common.hpp>
#include <boost/log/utility/setup/from_stream.hpp>

#include "../src/PID.h"

namespace logging = boost::log;
namespace attrs = boost::log::attributes;

using namespace std;
using namespace boost;
using namespace boost::unit_test;

BOOST_AUTO_TEST_SUITE(PIDTestSuite)

  struct TestFixture {
    TestFixture() {

      // INITIALIZE LOGGER

      // Open the log settings file
      std::string logFileSettingsName = "../../logs/settings_for_test.txt";
      std::ifstream settings(logFileSettingsName);
      if (!settings.is_open()) {
        std::cerr << "Could not open log settings file: " << logFileSettingsName << std::endl;
      }

      // Read the settings and initialize logging library
      logging::init_from_stream(settings);

      // LOGGER INITIALIZED

      BOOST_TEST_MESSAGE ("Setup Test Fixture");
    }

    ~TestFixture() {
      BOOST_TEST_MESSAGE ("Teardown Test Fixture");
    }

        int i;
    };

    BOOST_FIXTURE_TEST_CASE(PIDInitTest, TestFixture) {

      BOOST_TEST_MESSAGE ("Entering Init Test");

      PID pid;

      // Define PID gains
      double Kp = 0.1;      // Initial value for Kp
      double Ki = 0.001;    // Initial value for Ki
      double Kd = 2;      // Initial value for Kd

      // Define tuning variable
      bool do_tune = false;

      // Initialize PID
      pid.Init(Kp, Ki, Kd, do_tune);

      // Test Kp initialization
      BOOST_CHECK_MESSAGE(pid.GetKp() == Kp, "Kp Init Failed");

      // Test Ki initialization
      BOOST_CHECK_MESSAGE(pid.GetKi() == Ki, "Ki Init Failed");

      // Test Kd initialization
      BOOST_CHECK_MESSAGE(pid.GetKd() == Kd, "Kd Init Failed");

      // Test tuning flag initialization
      BOOST_CHECK_MESSAGE(pid.GetTuneFlag() == do_tune, "Tuning Flag Init Failed");

      BOOST_TEST_MESSAGE ("Leaving Init Test");
    }

    BOOST_FIXTURE_TEST_CASE(PIDSetTest, TestFixture) {

      BOOST_TEST_MESSAGE ("Entering Set Test");

      PID pid;

      // Define PID gains
      double Kp = 0.1;      // Initial value for Kp
      double Ki = 0.001;    // Initial value for Ki
      double Kd = 3.0;      // Initial value for Kd

      // Define tuning variable
      bool do_tune = false;

      // Initialize PID
      pid.Init(Kp, Ki, Kd, do_tune);

      // Set different PID gains
      pid.SetGains(2 * Kp, 2 * Ki, 2 * Kd);

      // Test Kp setting
      BOOST_CHECK_MESSAGE(pid.GetKp() == 2 * Kp, "Kp Setting Failed");

      // Test Ki setting
      BOOST_CHECK_MESSAGE(pid.GetKi() == 2 * Ki, "Ki Setting Failed");

      // Test Kd setting
      BOOST_CHECK_MESSAGE(pid.GetKd() == 2 * Kd, "Kd Setting Failed");

      BOOST_TEST_MESSAGE ("Leaving Set Test");
    }

    BOOST_FIXTURE_TEST_CASE(PIDErrorTest, TestFixture) {

      BOOST_TEST_MESSAGE ("Entering Error Calc Test");

      PID pid;

      // Define PID gains
      double Kp = 0.1;      // Initial value for Kp
      double Ki = 0.001;    // Initial value for Ki
      double Kd = 3;      // Initial value for Kd

      // Define tuning variable
      bool do_tune = false;

      // Initialize PID
      pid.Init(Kp, Ki, Kd, do_tune);

      // Define CTE
      double cte1 = 0.5;

      // Update error
      pid.UpdateError(cte1);

      // Test update error 1 - first update. All errors = cte
      BOOST_CHECK_MESSAGE(pid.OutputSteeringAngle() == -(Kp * cte1) - (Ki * cte1) - (Kd * cte1), "Steering Calc 1 "
                                                                                                 "Failed");
      // Define new CTE
      double cte2 = 0.3;

      // Update error
      pid.UpdateError(cte2);

      // Test update error 2 - generic update.
      BOOST_CHECK_MESSAGE(pid.OutputSteeringAngle() == -(Kp * cte2) - (Ki * (cte1 + cte2)) - (Kd * (cte2 - cte1)),
                          "Steering Calc 2 Failed");

      BOOST_TEST_MESSAGE ("Leaving Error Calc Test");
    }

    BOOST_FIXTURE_TEST_CASE(PIDTuningTest, TestFixture) {

      BOOST_TEST_MESSAGE ("Entering Tuning Test");

      PID pid;

      // Define PID gains
      double Kp = 0.1;      // Initial value for Kp
      double Ki = 0.001;    // Initial value for Ki
      double Kd = 3;        // Initial value for Kd

      // Define tuning variable
      bool do_tune = false;

      // Define tuning parameters
      int init_it = 0;
      int max_it = 1000;

      // Initialize PID
      pid.Init(Kp, Ki, Kd, do_tune, init_it, max_it);

     
      BOOST_TEST_MESSAGE ("Leaving Tuning Test");
    }

BOOST_AUTO_TEST_SUITE_END()


