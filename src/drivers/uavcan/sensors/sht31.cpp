
#include "sht31.hpp"

#include <drivers/drv_sht31.h>
#include <drivers/drv_hrt.h>
#include <lib/ecl/geo/geo.h>
#include <parameters/param.h>
#include <systemlib/err.h>


const char *const UavcanSht31Bridge::NAME = "sht31_sensor";

UavcanSht31Bridge::UavcanSht31Bridge(uavcan::INode &node) :
	UavcanSensorBridgeBase("uavcan_sht31_sensor", ORB_ID(sht31)),
	_sub_sht31(node)
{
}


int UavcanSht31Bridge::init()
{


	int res = _sub_sht31.start(Sht31CbBinder(this, &UavcanSht31Bridge::sht31_sub_cb));

	if (res < 0) {
		DEVICE_LOG("failed to start uavcan sub: %d", res);
		return res;
	}

	return 0;
}

void UavcanSht31Bridge::sht31_sub_cb(const
				     uavcan::ReceivedDataStructure<ardupilot::Sht31>
				     &msg)
{
	_device_id.devid_s.devtype = DRV_ATMOS_DEVTYPE_SHT31;
	_device_id.devid_s.address = msg.getSrcNodeID().get() & 0xFF;

	float humidity = msg.humidity;
	float temperature_c = msg.temperature + CONSTANTS_ABSOLUTE_NULL_CELSIUS;

	printf("msg.humidity = %f\n", (double)msg.humidity);
	printf("msg.temperature = %f\n", (double)msg.temperature);

	printf("humidity = %f\n", (double)humidity);
	printf("temperature_c = %f\n", (double)temperature_c);

	sht31_s report = {
		.timestamp = hrt_absolute_time(),
		.temperature = temperature_c,
		.humidity  = humidity,
		.device_id = _device_id.devid
	};



	publish(msg.getSrcNodeID().get(), &report);
}
