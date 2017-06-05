#include <boost/test/unit_test.hpp>
#include <motoman_mh12/Dummy.hpp>

using namespace motoman_mh12;

BOOST_AUTO_TEST_CASE(it_should_not_crash_when_welcome_is_called)
{
    motoman_mh12::DummyClass dummy;
    dummy.welcome();
}
