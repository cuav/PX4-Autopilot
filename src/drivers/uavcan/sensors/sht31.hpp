#include <uORB/topics/sht31.h>
#include "sensor_bridge.hpp"
#include <ardupilot/Sht31.hpp>


class UavcanSht31Bridge : public UavcanSensorBridgeBase
{
public:
	static const char *const NAME;

	UavcanSht31Bridge(uavcan::INode &node);

	const char *get_name() const override { return NAME; }

	int init() override;

private:
	float _temperature {0.0f};
	float _humidity    {0.0f};


	void sht31_sub_cb(const uavcan::ReceivedDataStructure<ardupilot::Sht31> &msg);

	typedef uavcan::MethodBinder < UavcanSht31Bridge *,
		void (UavcanSht31Bridge::*)
		(const uavcan::ReceivedDataStructure<ardupilot::Sht31> &) >
		Sht31CbBinder;

	uavcan::Subscriber<ardupilot::Sht31, Sht31CbBinder> _sub_sht31;
};



