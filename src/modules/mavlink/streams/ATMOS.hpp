#ifndef ATMOS_HPP
#define ATMOS_HPP

#include <uORB/topics/atmos.h>


class MavlinkStreamATMOS : public MavlinkStream
{
public:

	static MavlinkStream *new_instance(Mavlink *mavlink) { return new MavlinkStreamATMOS(mavlink); }

	static constexpr const char *get_name_static() { return "ATMOS_STATUS"; }
	static constexpr uint16_t get_id_static() { return MAVLINK_MSG_ID_ATMOSPHERE_OUTPUT_STATUS; }


	const char *get_name() const override { return get_name_static(); }
	uint16_t get_id() override { return get_id_static(); }

	unsigned get_size() override
	{
		return MAVLINK_MSG_ID_ATMOSPHERE_OUTPUT_STATUS_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES;
	}

private:
	explicit MavlinkStreamATMOS(Mavlink *mavlink) : MavlinkStream(mavlink) {}

	uORB::Subscription _atmos_sub{ORB_ID(atmos)};

	/* do not allow top copying this class */
	MavlinkStreamATMOS(MavlinkStreamATMOS &) = delete;
	MavlinkStreamATMOS &operator = (const MavlinkStreamATMOS &) = delete;

	bool send() override
	{
		atmos_s atmos{};
		mavlink_atmosphere_output_status_t msg;
		_atmos_sub.copy(&atmos);
		msg.atmosphere_temp = atmos.temperature;
		msg.atmosphere_humidity = atmos.humidity;

		mavlink_msg_atmosphere_output_status_send_struct(_mavlink->get_channel(), &msg);

		return true;
	}
};


#endif
