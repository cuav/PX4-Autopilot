#ifndef HYGROMETER_HPP
#define HYGROMETER_HPP

#include <uORB/topics/hygrometer.h>


class MavlinkStreamHYGRO : public MavlinkStream
{
public:

	static MavlinkStream *new_instance(Mavlink *mavlink) { return new MavlinkStreamHYGRO(mavlink); }

	static constexpr const char *get_name_static() { return "HYGROMETER_SENSOR"; }
	static constexpr uint16_t get_id_static() { return MAVLINK_MSG_ID_HYGROMETER_SENSOR; }


	const char *get_name() const override { return get_name_static(); }
	uint16_t get_id() override { return get_id_static(); }

	unsigned get_size() override
	{
		return MAVLINK_MSG_ID_HYGROMETER_SENSOR_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES;
	}

private:
	explicit MavlinkStreamHYGRO(Mavlink *mavlink) : MavlinkStream(mavlink) {}

	uORB::Subscription _hygrometer_sub{ORB_ID(hygrometer)};

	/* do not allow top copying this class */
	MavlinkStreamHYGRO(MavlinkStreamHYGRO &) = delete;
	MavlinkStreamHYGRO &operator = (const MavlinkStreamHYGRO &) = delete;

	bool send() override
	{
		hygrometer_s hygrometer{};
		mavlink_hygrometer_sensor_t msg;
		_hygrometer_sub.copy(&hygrometer);
		msg.temperature = hygrometer.temperature;
		msg.humidity = hygrometer.humidity;
		msg.id = hygrometer.id;

		mavlink_msg_hygrometer_sensor_send_struct(_mavlink->get_channel(), &msg);

		return true;
	}
};


#endif
