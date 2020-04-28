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
      bool do_tune = true;

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
      double Kd = 3.0;      // Initial value for Kd

      // Define tuning variable
      bool do_tune = true;

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
      double Kd = 3.0;      // Initial value for Kd

      // Define tuning variable
      bool do_tune = true;

      // Define tuning parameters
      int init_it = 0;
      int max_it = 1000;

      // Initialize PID
      pid.Init(Kp, Ki, Kd, do_tune, init_it, max_it);

      // Define CTE
      double cte = 0.5;

      // Call update error - this will also increment the iteration counter
      pid.UpdateError(cte);

      BOOST_TEST_MESSAGE ("Tuning Test Step 1");

      // Step 1.1 - First call on first p[] index. This will increase Kp by ten 10%
      pid.TuneGains();

      // Test Tuning step 1.1 - Kp increased by 10%
      BOOST_CHECK_MESSAGE(fabs(pid.GetKp() - 1.1*Kp) < 1e-12, "Tuning step 1.1 (Kp first increment) failed");

      // Call update error / increment iteration counter
      pid.UpdateError(cte);

      // Step 1.2 - This will be tuning case 1, and Kp best error will change. Kp should stay = 1.1 the initial value
      pid.TuneGains();

      // Test Tuning step 1.2 - Kp stays at 1.1 initial value
      BOOST_CHECK_MESSAGE(fabs(pid.GetKp() - 1.1*Kp) < 1e-12, "Tuning step 1.2 (Kp increment confirmed) failed");

      // Call update error / increment iteration counter
      pid.UpdateError(cte);

      // Step 1.3 - First call on second p[] index. This will increase Ki by ten 10%
      pid.TuneGains();

      // Test Tuning step 1.3 - Ki increased by 10%
      BOOST_CHECK_MESSAGE(fabs(pid.GetKi() - 1.1*Ki) < 1e-12, "Tuning step 1.3 (Ki first increment) failed");

      // Decrease cte to force Tuning case 1
      cte = 0.49;

      // Call update error / increment iteration counter
      pid.UpdateError(cte);

      // Step 1.4 - This will be tuning case 1, and Ki best error will change. Ki should stay = 1.1 the initial value
      pid.TuneGains();

      // Test Tuning step 1.4 - Ki stays at 1.1 initial value
      BOOST_CHECK_MESSAGE(fabs(pid.GetKi() - 1.1*Ki) < 1e-12, "Tuning step 1.4 (Ki increment confirmed) failed");

      // Call update error / increment iteration counter
      pid.UpdateError(cte);

      // Step 1.5 - First call on third p[] index. This will increase Kd by ten 10%
      pid.TuneGains();

      // Test Tuning step 1.5 - Kd increased by 10%
      BOOST_CHECK_MESSAGE(fabs(pid.GetKd() - 1.1*Kd) < 1e-12, "Tuning step 1.5 (Kd first increment) failed");

      // Decrease cte to force Tuning case 1
      cte = 0.48;

      // Call update error / increment iteration counter
      pid.UpdateError(cte);

      // Step 1.6 - This will be tuning case 1, and Kd best error will change. Kd should stay = 1.1 the initial value
      pid.TuneGains();

      // Test Tuning step 1.6 - Kd stays at 1.1 initial value
      BOOST_CHECK_MESSAGE(fabs(pid.GetKd() - 1.1*Kd) < 1e-12, "Tuning step 1.6 (Kd increment confirmed) failed");

      BOOST_TEST_MESSAGE ("Tuning Test Step 2");

      // Reset gains and errors
      Kp = pid.GetKp();
      Ki = pid.GetKi();
      Kd = pid.GetKd();
      cte = 0.5;

      // Call update error / increment iteration counter
      pid.UpdateError(cte);

      // Step 2.1- This will be tuning case 2: error increased and Kp should be = 0.9 of the initial value
      pid.TuneGains();

      // Test Tuning step 2.1 - Kp is decremented to 0.9 initial value
      BOOST_CHECK_MESSAGE(fabs(pid.GetKp() - 1.1*Kp) < 1e-12, "Tuning step 2.1 (Kp first increment) failed");

      // Increase cte to force Tuning case 1
      cte = 0.51;

      // Call update error / increment iteration counter
      pid.UpdateError(cte);

      // Step 2.2 - This will be tuning case 2, and Kp best error will change. Kp should go = 0.9 of the initial value
      pid.TuneGains();

      // Test Tuning step 2.1 - Kp is decremented to 0.9 initial value
      BOOST_CHECK_MESSAGE(fabs(pid.GetKp() - 0.9*Kp) < 1e-12, "Tuning step 2.1 (Kp first decrement) failed");


      BOOST_TEST_MESSAGE ("Leaving Tuning Test");
    }

BOOST_AUTO_TEST_SUITE_END()


