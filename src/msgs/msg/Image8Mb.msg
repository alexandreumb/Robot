int64 timestamp
int64 publish_timestamp
uint8 is_bigendian
uint32 frequency
uint32 step_color
uint8[3145728] data_color  # 12582912 = 2048 * 2048 * 3
uint32 step_depth
uint8[3145728] data_depth  # 12582912 = 2048 * 2048 * 3
ImageIntrinsics image_intrinsics