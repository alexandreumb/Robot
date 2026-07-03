int64 timestamp
int64 publish_timestamp
uint8 is_bigendian
uint32 frequency
uint32 step_color
uint8[3145728] data_color  # 1024* 1024 * 3
uint32 step_depth
uint8[2097152] data_depth  # 1024* 1024 * 2
ImageIntrinsics image_intrinsics