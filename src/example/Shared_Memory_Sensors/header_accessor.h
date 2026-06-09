#ifndef HEADER_ACCESSOR_H
#define HEADER_ACCESSOR_H


#include <cassert>
#include <boost/interprocess/shared_memory_object.hpp>
#include <boost/interprocess/mapped_region.hpp>
#include <memory>
struct gps_reading
{	int counter;
	double latitude;
	double longitude;
	double height;
	double velocity[3];
	double acceleration[3];
	double gforce;
	double orientation[3];
	double angular_velocity[3];
	double standard_deviation[3];
};

struct gps_reading_layout 
{	 size_t counter_address = 0;
	 size_t counter_size = 4;

	 size_t latitude_address = 4;
	 size_t latitude_size = 8;

	 size_t longitude_address = 12;
	 size_t longitude_size = 8;

	 size_t height_address = 20;
	 size_t height_size = 8;

	 size_t velocity_address = 28;
	 size_t velocity_size = 24;

	 size_t acceleration_address = 52;
	 size_t acceleration_size = 24;

	 size_t gforce_address = 76;
	 size_t gforce_size = 8;

	 size_t orientation_address = 84;
	 size_t orientation_size = 24;

	 size_t angular_velocity_address = 108;
	 size_t angular_velocity_size = 24;

	 size_t standard_deviation_address = 132;
	 size_t standard_deviation_size = 24;

};

void copy_from_gps_reading_to_shared_memory(unsigned char* memory, const gps_reading & tmp)
{ 
	constexpr gps_reading_layout mapping;
	std::memcpy(memory + mapping.counter_address, &tmp.counter, mapping.counter_size);
	std::memcpy(memory + mapping.latitude_address, &tmp.latitude, mapping.latitude_size);
	std::memcpy(memory + mapping.longitude_address, &tmp.longitude, mapping.longitude_size);
	std::memcpy(memory + mapping.height_address, &tmp.height, mapping.height_size);
	std::memcpy(memory + mapping.velocity_address, &tmp.velocity, mapping.velocity_size);
	std::memcpy(memory + mapping.acceleration_address, &tmp.acceleration, mapping.acceleration_size);
	std::memcpy(memory + mapping.gforce_address, &tmp.gforce, mapping.gforce_size);
	std::memcpy(memory + mapping.orientation_address, &tmp.orientation, mapping.orientation_size);
	std::memcpy(memory + mapping.angular_velocity_address, &tmp.angular_velocity, mapping.angular_velocity_size);
	std::memcpy(memory + mapping.standard_deviation_address, &tmp.standard_deviation, mapping.standard_deviation_size);
}
void copy_from_shared_memory_to_gps_reading(const unsigned char* memory, gps_reading & tmp)
{ 
	constexpr gps_reading_layout mapping;
	std::memcpy(&tmp.counter, memory + mapping.counter_address, mapping.counter_size);
	std::memcpy(&tmp.latitude, memory + mapping.latitude_address, mapping.latitude_size);
	std::memcpy(&tmp.longitude, memory + mapping.longitude_address, mapping.longitude_size);
	std::memcpy(&tmp.height, memory + mapping.height_address, mapping.height_size);
	std::memcpy(&tmp.velocity, memory + mapping.velocity_address, mapping.velocity_size);
	std::memcpy(&tmp.acceleration, memory + mapping.acceleration_address, mapping.acceleration_size);
	std::memcpy(&tmp.gforce, memory + mapping.gforce_address, mapping.gforce_size);
	std::memcpy(&tmp.orientation, memory + mapping.orientation_address, mapping.orientation_size);
	std::memcpy(&tmp.angular_velocity, memory + mapping.angular_velocity_address, mapping.angular_velocity_size);
	std::memcpy(&tmp.standard_deviation, memory + mapping.standard_deviation_address, mapping.standard_deviation_size);
}
struct image_reading
{	int counter;
	unsigned char* color_data = nullptr;
	unsigned char* depth_data = nullptr;
};

struct image_reading_layout 
{	 size_t counter_address = 156;
	 size_t counter_size = 4;

	 size_t color_data_address = 160;
	 size_t color_data_size = 2764800;

	 size_t depth_data_address = 2764960;
	 size_t depth_data_size = 1843200;

};

void copy_from_image_reading_to_shared_memory(unsigned char* memory, const image_reading & tmp)
{ 
	constexpr image_reading_layout mapping;
	std::memcpy(memory + mapping.counter_address, &tmp.counter, mapping.counter_size);
	assert(tmp.color_data != nullptr);
	std::memcpy(memory + mapping.color_data_address, tmp.color_data, mapping.color_data_size);
	assert(tmp.depth_data != nullptr);
	std::memcpy(memory + mapping.depth_data_address, tmp.depth_data, mapping.depth_data_size);
}
void copy_from_shared_memory_to_image_reading(const unsigned char* memory, image_reading & tmp)
{ 
	constexpr image_reading_layout mapping;
	std::memcpy(&tmp.counter, memory + mapping.counter_address, mapping.counter_size);
	assert(tmp.color_data != nullptr);
	std::memcpy(tmp.color_data, memory + mapping.color_data_address, mapping.color_data_size);
	assert(tmp.depth_data != nullptr);
	std::memcpy(tmp.depth_data, memory + mapping.depth_data_address, mapping.depth_data_size);
}
struct image_intrinsics
{	int width;
	int height;
	float fx;
	float fy;
	float ppx;
	float ppy;
	float depth_units;
};

struct image_intrinsics_layout 
{	 size_t width_address = 4608160;
	 size_t width_size = 4;

	 size_t height_address = 4608164;
	 size_t height_size = 4;

	 size_t fx_address = 4608168;
	 size_t fx_size = 4;

	 size_t fy_address = 4608172;
	 size_t fy_size = 4;

	 size_t ppx_address = 4608176;
	 size_t ppx_size = 4;

	 size_t ppy_address = 4608180;
	 size_t ppy_size = 4;

	 size_t depth_units_address = 4608184;
	 size_t depth_units_size = 4;

};

void copy_from_image_intrinsics_to_shared_memory(unsigned char* memory, const image_intrinsics & tmp)
{ 
	constexpr image_intrinsics_layout mapping;
	std::memcpy(memory + mapping.width_address, &tmp.width, mapping.width_size);
	std::memcpy(memory + mapping.height_address, &tmp.height, mapping.height_size);
	std::memcpy(memory + mapping.fx_address, &tmp.fx, mapping.fx_size);
	std::memcpy(memory + mapping.fy_address, &tmp.fy, mapping.fy_size);
	std::memcpy(memory + mapping.ppx_address, &tmp.ppx, mapping.ppx_size);
	std::memcpy(memory + mapping.ppy_address, &tmp.ppy, mapping.ppy_size);
	std::memcpy(memory + mapping.depth_units_address, &tmp.depth_units, mapping.depth_units_size);
}
void copy_from_shared_memory_to_image_intrinsics(const unsigned char* memory, image_intrinsics & tmp)
{ 
	constexpr image_intrinsics_layout mapping;
	std::memcpy(&tmp.width, memory + mapping.width_address, mapping.width_size);
	std::memcpy(&tmp.height, memory + mapping.height_address, mapping.height_size);
	std::memcpy(&tmp.fx, memory + mapping.fx_address, mapping.fx_size);
	std::memcpy(&tmp.fy, memory + mapping.fy_address, mapping.fy_size);
	std::memcpy(&tmp.ppx, memory + mapping.ppx_address, mapping.ppx_size);
	std::memcpy(&tmp.ppy, memory + mapping.ppy_address, mapping.ppy_size);
	std::memcpy(&tmp.depth_units, memory + mapping.depth_units_address, mapping.depth_units_size);
}
struct can_reading
{	int counter;
	int temp_front_left;
	int temp_front_right;
	int temp_front_timestamp;
	int temp_rear_left;
	int temp_rear_right;
	int temp_rear_timestamp;
	int velocity_front_left;
	int velocity_front_right;
	int velocity_front_timestamp;
	int velocity_rear_left;
	int velocity_rear_right;
	int velocity_rear_timestamp;
	int throttle_front_left;
	int throttle_front_right;
	int throttle_front_timestamp;
	int throttle_rear_left;
	int throttle_rear_right;
	int throttle_rear_timestamp;
	int reverse_front_left;
	int reverse_front_right;
	int reverse_front_timestamp;
	int reverse_rear_left;
	int reverse_rear_right;
	int reverse_rear_timestamp;
	int brake_front_left;
	int brake_front_right;
	int brake_front_timestamp;
	int brake_rear_left;
	int brake_rear_right;
	int brake_rear_timestamp;
	int steering_angle;
	int steering_angle_timestamp;
	int steering_torque_high;
	int steering_torque_low;
	int steering_torque_timestamp;
	int battery_voltage;
	int battery_voltage_timestamp;
	int controllers_status;
	int controllers_status_timestamp;
};

struct can_reading_layout 
{	 size_t counter_address = 4608188;
	 size_t counter_size = 4;

	 size_t temp_front_left_address = 4608192;
	 size_t temp_front_left_size = 4;

	 size_t temp_front_right_address = 4608196;
	 size_t temp_front_right_size = 4;

	 size_t temp_front_timestamp_address = 4608200;
	 size_t temp_front_timestamp_size = 4;

	 size_t temp_rear_left_address = 4608204;
	 size_t temp_rear_left_size = 4;

	 size_t temp_rear_right_address = 4608208;
	 size_t temp_rear_right_size = 4;

	 size_t temp_rear_timestamp_address = 4608212;
	 size_t temp_rear_timestamp_size = 4;

	 size_t velocity_front_left_address = 4608216;
	 size_t velocity_front_left_size = 4;

	 size_t velocity_front_right_address = 4608220;
	 size_t velocity_front_right_size = 4;

	 size_t velocity_front_timestamp_address = 4608224;
	 size_t velocity_front_timestamp_size = 4;

	 size_t velocity_rear_left_address = 4608228;
	 size_t velocity_rear_left_size = 4;

	 size_t velocity_rear_right_address = 4608232;
	 size_t velocity_rear_right_size = 4;

	 size_t velocity_rear_timestamp_address = 4608236;
	 size_t velocity_rear_timestamp_size = 4;

	 size_t throttle_front_left_address = 4608240;
	 size_t throttle_front_left_size = 4;

	 size_t throttle_front_right_address = 4608244;
	 size_t throttle_front_right_size = 4;

	 size_t throttle_front_timestamp_address = 4608248;
	 size_t throttle_front_timestamp_size = 4;

	 size_t throttle_rear_left_address = 4608252;
	 size_t throttle_rear_left_size = 4;

	 size_t throttle_rear_right_address = 4608256;
	 size_t throttle_rear_right_size = 4;

	 size_t throttle_rear_timestamp_address = 4608260;
	 size_t throttle_rear_timestamp_size = 4;

	 size_t reverse_front_left_address = 4608264;
	 size_t reverse_front_left_size = 4;

	 size_t reverse_front_right_address = 4608268;
	 size_t reverse_front_right_size = 4;

	 size_t reverse_front_timestamp_address = 4608272;
	 size_t reverse_front_timestamp_size = 4;

	 size_t reverse_rear_left_address = 4608276;
	 size_t reverse_rear_left_size = 4;

	 size_t reverse_rear_right_address = 4608280;
	 size_t reverse_rear_right_size = 4;

	 size_t reverse_rear_timestamp_address = 4608284;
	 size_t reverse_rear_timestamp_size = 4;

	 size_t brake_front_left_address = 4608288;
	 size_t brake_front_left_size = 4;

	 size_t brake_front_right_address = 4608292;
	 size_t brake_front_right_size = 4;

	 size_t brake_front_timestamp_address = 4608296;
	 size_t brake_front_timestamp_size = 4;

	 size_t brake_rear_left_address = 4608300;
	 size_t brake_rear_left_size = 4;

	 size_t brake_rear_right_address = 4608304;
	 size_t brake_rear_right_size = 4;

	 size_t brake_rear_timestamp_address = 4608308;
	 size_t brake_rear_timestamp_size = 4;

	 size_t steering_angle_address = 4608312;
	 size_t steering_angle_size = 4;

	 size_t steering_angle_timestamp_address = 4608316;
	 size_t steering_angle_timestamp_size = 4;

	 size_t steering_torque_high_address = 4608320;
	 size_t steering_torque_high_size = 4;

	 size_t steering_torque_low_address = 4608324;
	 size_t steering_torque_low_size = 4;

	 size_t steering_torque_timestamp_address = 4608328;
	 size_t steering_torque_timestamp_size = 4;

	 size_t battery_voltage_address = 4608332;
	 size_t battery_voltage_size = 4;

	 size_t battery_voltage_timestamp_address = 4608336;
	 size_t battery_voltage_timestamp_size = 4;

	 size_t controllers_status_address = 4608340;
	 size_t controllers_status_size = 4;

	 size_t controllers_status_timestamp_address = 4608344;
	 size_t controllers_status_timestamp_size = 4;

};

void copy_from_can_reading_to_shared_memory(unsigned char* memory, const can_reading & tmp)
{ 
	constexpr can_reading_layout mapping;
	std::memcpy(memory + mapping.counter_address, &tmp.counter, mapping.counter_size);
	std::memcpy(memory + mapping.temp_front_left_address, &tmp.temp_front_left, mapping.temp_front_left_size);
	std::memcpy(memory + mapping.temp_front_right_address, &tmp.temp_front_right, mapping.temp_front_right_size);
	std::memcpy(memory + mapping.temp_front_timestamp_address, &tmp.temp_front_timestamp, mapping.temp_front_timestamp_size);
	std::memcpy(memory + mapping.temp_rear_left_address, &tmp.temp_rear_left, mapping.temp_rear_left_size);
	std::memcpy(memory + mapping.temp_rear_right_address, &tmp.temp_rear_right, mapping.temp_rear_right_size);
	std::memcpy(memory + mapping.temp_rear_timestamp_address, &tmp.temp_rear_timestamp, mapping.temp_rear_timestamp_size);
	std::memcpy(memory + mapping.velocity_front_left_address, &tmp.velocity_front_left, mapping.velocity_front_left_size);
	std::memcpy(memory + mapping.velocity_front_right_address, &tmp.velocity_front_right, mapping.velocity_front_right_size);
	std::memcpy(memory + mapping.velocity_front_timestamp_address, &tmp.velocity_front_timestamp, mapping.velocity_front_timestamp_size);
	std::memcpy(memory + mapping.velocity_rear_left_address, &tmp.velocity_rear_left, mapping.velocity_rear_left_size);
	std::memcpy(memory + mapping.velocity_rear_right_address, &tmp.velocity_rear_right, mapping.velocity_rear_right_size);
	std::memcpy(memory + mapping.velocity_rear_timestamp_address, &tmp.velocity_rear_timestamp, mapping.velocity_rear_timestamp_size);
	std::memcpy(memory + mapping.throttle_front_left_address, &tmp.throttle_front_left, mapping.throttle_front_left_size);
	std::memcpy(memory + mapping.throttle_front_right_address, &tmp.throttle_front_right, mapping.throttle_front_right_size);
	std::memcpy(memory + mapping.throttle_front_timestamp_address, &tmp.throttle_front_timestamp, mapping.throttle_front_timestamp_size);
	std::memcpy(memory + mapping.throttle_rear_left_address, &tmp.throttle_rear_left, mapping.throttle_rear_left_size);
	std::memcpy(memory + mapping.throttle_rear_right_address, &tmp.throttle_rear_right, mapping.throttle_rear_right_size);
	std::memcpy(memory + mapping.throttle_rear_timestamp_address, &tmp.throttle_rear_timestamp, mapping.throttle_rear_timestamp_size);
	std::memcpy(memory + mapping.reverse_front_left_address, &tmp.reverse_front_left, mapping.reverse_front_left_size);
	std::memcpy(memory + mapping.reverse_front_right_address, &tmp.reverse_front_right, mapping.reverse_front_right_size);
	std::memcpy(memory + mapping.reverse_front_timestamp_address, &tmp.reverse_front_timestamp, mapping.reverse_front_timestamp_size);
	std::memcpy(memory + mapping.reverse_rear_left_address, &tmp.reverse_rear_left, mapping.reverse_rear_left_size);
	std::memcpy(memory + mapping.reverse_rear_right_address, &tmp.reverse_rear_right, mapping.reverse_rear_right_size);
	std::memcpy(memory + mapping.reverse_rear_timestamp_address, &tmp.reverse_rear_timestamp, mapping.reverse_rear_timestamp_size);
	std::memcpy(memory + mapping.brake_front_left_address, &tmp.brake_front_left, mapping.brake_front_left_size);
	std::memcpy(memory + mapping.brake_front_right_address, &tmp.brake_front_right, mapping.brake_front_right_size);
	std::memcpy(memory + mapping.brake_front_timestamp_address, &tmp.brake_front_timestamp, mapping.brake_front_timestamp_size);
	std::memcpy(memory + mapping.brake_rear_left_address, &tmp.brake_rear_left, mapping.brake_rear_left_size);
	std::memcpy(memory + mapping.brake_rear_right_address, &tmp.brake_rear_right, mapping.brake_rear_right_size);
	std::memcpy(memory + mapping.brake_rear_timestamp_address, &tmp.brake_rear_timestamp, mapping.brake_rear_timestamp_size);
	std::memcpy(memory + mapping.steering_angle_address, &tmp.steering_angle, mapping.steering_angle_size);
	std::memcpy(memory + mapping.steering_angle_timestamp_address, &tmp.steering_angle_timestamp, mapping.steering_angle_timestamp_size);
	std::memcpy(memory + mapping.steering_torque_high_address, &tmp.steering_torque_high, mapping.steering_torque_high_size);
	std::memcpy(memory + mapping.steering_torque_low_address, &tmp.steering_torque_low, mapping.steering_torque_low_size);
	std::memcpy(memory + mapping.steering_torque_timestamp_address, &tmp.steering_torque_timestamp, mapping.steering_torque_timestamp_size);
	std::memcpy(memory + mapping.battery_voltage_address, &tmp.battery_voltage, mapping.battery_voltage_size);
	std::memcpy(memory + mapping.battery_voltage_timestamp_address, &tmp.battery_voltage_timestamp, mapping.battery_voltage_timestamp_size);
	std::memcpy(memory + mapping.controllers_status_address, &tmp.controllers_status, mapping.controllers_status_size);
	std::memcpy(memory + mapping.controllers_status_timestamp_address, &tmp.controllers_status_timestamp, mapping.controllers_status_timestamp_size);
}
void copy_from_shared_memory_to_can_reading(const unsigned char* memory, can_reading & tmp)
{ 
	constexpr can_reading_layout mapping;
	std::memcpy(&tmp.counter, memory + mapping.counter_address, mapping.counter_size);
	std::memcpy(&tmp.temp_front_left, memory + mapping.temp_front_left_address, mapping.temp_front_left_size);
	std::memcpy(&tmp.temp_front_right, memory + mapping.temp_front_right_address, mapping.temp_front_right_size);
	std::memcpy(&tmp.temp_front_timestamp, memory + mapping.temp_front_timestamp_address, mapping.temp_front_timestamp_size);
	std::memcpy(&tmp.temp_rear_left, memory + mapping.temp_rear_left_address, mapping.temp_rear_left_size);
	std::memcpy(&tmp.temp_rear_right, memory + mapping.temp_rear_right_address, mapping.temp_rear_right_size);
	std::memcpy(&tmp.temp_rear_timestamp, memory + mapping.temp_rear_timestamp_address, mapping.temp_rear_timestamp_size);
	std::memcpy(&tmp.velocity_front_left, memory + mapping.velocity_front_left_address, mapping.velocity_front_left_size);
	std::memcpy(&tmp.velocity_front_right, memory + mapping.velocity_front_right_address, mapping.velocity_front_right_size);
	std::memcpy(&tmp.velocity_front_timestamp, memory + mapping.velocity_front_timestamp_address, mapping.velocity_front_timestamp_size);
	std::memcpy(&tmp.velocity_rear_left, memory + mapping.velocity_rear_left_address, mapping.velocity_rear_left_size);
	std::memcpy(&tmp.velocity_rear_right, memory + mapping.velocity_rear_right_address, mapping.velocity_rear_right_size);
	std::memcpy(&tmp.velocity_rear_timestamp, memory + mapping.velocity_rear_timestamp_address, mapping.velocity_rear_timestamp_size);
	std::memcpy(&tmp.throttle_front_left, memory + mapping.throttle_front_left_address, mapping.throttle_front_left_size);
	std::memcpy(&tmp.throttle_front_right, memory + mapping.throttle_front_right_address, mapping.throttle_front_right_size);
	std::memcpy(&tmp.throttle_front_timestamp, memory + mapping.throttle_front_timestamp_address, mapping.throttle_front_timestamp_size);
	std::memcpy(&tmp.throttle_rear_left, memory + mapping.throttle_rear_left_address, mapping.throttle_rear_left_size);
	std::memcpy(&tmp.throttle_rear_right, memory + mapping.throttle_rear_right_address, mapping.throttle_rear_right_size);
	std::memcpy(&tmp.throttle_rear_timestamp, memory + mapping.throttle_rear_timestamp_address, mapping.throttle_rear_timestamp_size);
	std::memcpy(&tmp.reverse_front_left, memory + mapping.reverse_front_left_address, mapping.reverse_front_left_size);
	std::memcpy(&tmp.reverse_front_right, memory + mapping.reverse_front_right_address, mapping.reverse_front_right_size);
	std::memcpy(&tmp.reverse_front_timestamp, memory + mapping.reverse_front_timestamp_address, mapping.reverse_front_timestamp_size);
	std::memcpy(&tmp.reverse_rear_left, memory + mapping.reverse_rear_left_address, mapping.reverse_rear_left_size);
	std::memcpy(&tmp.reverse_rear_right, memory + mapping.reverse_rear_right_address, mapping.reverse_rear_right_size);
	std::memcpy(&tmp.reverse_rear_timestamp, memory + mapping.reverse_rear_timestamp_address, mapping.reverse_rear_timestamp_size);
	std::memcpy(&tmp.brake_front_left, memory + mapping.brake_front_left_address, mapping.brake_front_left_size);
	std::memcpy(&tmp.brake_front_right, memory + mapping.brake_front_right_address, mapping.brake_front_right_size);
	std::memcpy(&tmp.brake_front_timestamp, memory + mapping.brake_front_timestamp_address, mapping.brake_front_timestamp_size);
	std::memcpy(&tmp.brake_rear_left, memory + mapping.brake_rear_left_address, mapping.brake_rear_left_size);
	std::memcpy(&tmp.brake_rear_right, memory + mapping.brake_rear_right_address, mapping.brake_rear_right_size);
	std::memcpy(&tmp.brake_rear_timestamp, memory + mapping.brake_rear_timestamp_address, mapping.brake_rear_timestamp_size);
	std::memcpy(&tmp.steering_angle, memory + mapping.steering_angle_address, mapping.steering_angle_size);
	std::memcpy(&tmp.steering_angle_timestamp, memory + mapping.steering_angle_timestamp_address, mapping.steering_angle_timestamp_size);
	std::memcpy(&tmp.steering_torque_high, memory + mapping.steering_torque_high_address, mapping.steering_torque_high_size);
	std::memcpy(&tmp.steering_torque_low, memory + mapping.steering_torque_low_address, mapping.steering_torque_low_size);
	std::memcpy(&tmp.steering_torque_timestamp, memory + mapping.steering_torque_timestamp_address, mapping.steering_torque_timestamp_size);
	std::memcpy(&tmp.battery_voltage, memory + mapping.battery_voltage_address, mapping.battery_voltage_size);
	std::memcpy(&tmp.battery_voltage_timestamp, memory + mapping.battery_voltage_timestamp_address, mapping.battery_voltage_timestamp_size);
	std::memcpy(&tmp.controllers_status, memory + mapping.controllers_status_address, mapping.controllers_status_size);
	std::memcpy(&tmp.controllers_status_timestamp, memory + mapping.controllers_status_timestamp_address, mapping.controllers_status_timestamp_size);
}
struct control_action
{	int counter;
	int actuation_direction;
	int actuation_velocity[4];
};

struct control_action_layout 
{	 size_t counter_address = 4608348;
	 size_t counter_size = 4;

	 size_t actuation_direction_address = 4608352;
	 size_t actuation_direction_size = 4;

	 size_t actuation_velocity_address = 4608356;
	 size_t actuation_velocity_size = 16;

};

void copy_from_control_action_to_shared_memory(unsigned char* memory, const control_action & tmp)
{ 
	constexpr control_action_layout mapping;
	std::memcpy(memory + mapping.counter_address, &tmp.counter, mapping.counter_size);
	std::memcpy(memory + mapping.actuation_direction_address, &tmp.actuation_direction, mapping.actuation_direction_size);
	std::memcpy(memory + mapping.actuation_velocity_address, &tmp.actuation_velocity, mapping.actuation_velocity_size);
}
void copy_from_shared_memory_to_control_action(const unsigned char* memory, control_action & tmp)
{ 
	constexpr control_action_layout mapping;
	std::memcpy(&tmp.counter, memory + mapping.counter_address, mapping.counter_size);
	std::memcpy(&tmp.actuation_direction, memory + mapping.actuation_direction_address, mapping.actuation_direction_size);
	std::memcpy(&tmp.actuation_velocity, memory + mapping.actuation_velocity_address, mapping.actuation_velocity_size);
}
struct SharedMemoryAccessor{
private:
	boost::interprocess::shared_memory_object shm;
	boost::interprocess::mapped_region region;
	explicit SharedMemoryAccessor() : shm{boost::interprocess::open_only, "KAZAMAS", boost::interprocess::read_write} {
		region = boost::interprocess::mapped_region{shm, boost::interprocess::read_write};
	}

public:
	static std::unique_ptr<SharedMemoryAccessor> create() {
		std::unique_ptr<SharedMemoryAccessor> unique = std::unique_ptr<SharedMemoryAccessor>(new SharedMemoryAccessor{});
		return unique;
	}

	~SharedMemoryAccessor() {}

	unsigned char* get_shared_memory_address() {
		return static_cast<unsigned char*>(region.get_address());
	}
	inline size_t size() {
		return 4608372;
	}
};

#endif // HEADER_ACCESSOR_H
