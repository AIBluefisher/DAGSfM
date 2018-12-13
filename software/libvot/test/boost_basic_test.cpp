// Author: Tianwei Shen <shentianweipku@gmail.com>
// This is adapted from Boost serialization example

#include <iostream>
#include <fstream>
#include <string>

// include headers that implement a archive in simple text format
#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <boost/serialization/version.hpp>

// a member class to test pointer serialization
class gps_device {
public:
	gps_device() {}
	gps_device(std::string name_): name(name_) {}
	std::string name;
};

class gps_position
{
public:
    int degrees;
    int minutes;
    float seconds;
	gps_device *device;		// pointer to the device that collect gps data
	gps_position() {}
    gps_position(int d, int m, float s, gps_device *dev) :
        degrees(d), minutes(m), seconds(s), device(dev)
    {}
	const std::string get_device_name() const
	{
		return device->name;
	}
};

// a non-intrusive way for serialization with boost
namespace boost {
namespace serialization {

template<class Archive>
void serialize(Archive & ar, gps_device & g, const unsigned int version)
{
	ar & g.name;
}

template<class Archive>
void serialize(Archive & ar, gps_position & g, const unsigned int version)
{
    ar & g.degrees;
    ar & g.minutes;
    ar & g.seconds;
	if(version > 0)
		ar & g.device;
}

} // namespace serialization
} // namespace boost

BOOST_CLASS_VERSION(gps_position, 1)

int main() {
    // create and open a character archive for output
    std::ofstream ofs("filename");

    // create class instance
	std::string name = "gps_device";
	gps_device d(name);
    const gps_position g(35, 59, 24.567f, &d);

    // save data to archive
    {
        boost::archive::text_oarchive oa(ofs);
        // write class instance to archive
        oa << g;
    	// archive and stream closed when destructors are called
    }

    // ... some time later restore the class instance to its orginal state
    gps_position newg;
    {
        // create and open an archive for input
        std::ifstream ifs("filename");
        boost::archive::text_iarchive ia(ifs);
        // read class state from archive
        ia >> newg;
        // archive and stream closed when destructors are called
    }
	const std::string device_name = newg.get_device_name();
	if(device_name != "gps_device")
		return -1;

    return 0;
}
